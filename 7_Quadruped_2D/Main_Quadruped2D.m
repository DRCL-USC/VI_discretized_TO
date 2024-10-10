clc; close all; clear
%% Add CASADI to path
filePath = matlab.desktop.editor.getActiveFilename;
fileParts = strsplit(filePath, filesep);
parentFile = fullfile(fileParts{1:end-1});
utilitiesFile = fullfile(fileParts{1:end-2}, 'Utilities');
cd(utilitiesFile);

if computer == 'PCWIN64'
     addpath(genpath('casadi-windows-matlabR2016a-v3.5.1')); % for Windows
else
    addpath(genpath('casadi')); % for Linux
end  

addpath(genpath('Functions'));

currentPATH = getenv("PATH");
MA27PATH = fullfile(pwd, 'CoinHSL-archive.v2023.11.17.x86_64-w64-mingw32-libgfortran5', 'bin');
newPATH = [currentPATH, ';', MA27PATH];
setenv("PATH", newPATH);

%Set Current Folder to Folder of Main Function
cd(parentFile);

%% Robot Parameters and Dynamics Generation 
disp_box('Generating Dynamics')

[nx, robotStruct, robotParams] = getA1Struct2D();
genDynamics(nx, robotStruct); %Generate Dynamics!

%% Integration Settings
disp_box('Initializing Trajectory Optimization')
tic %start timing setup time

%Choose Integration Method (Euler or VI)
INTEGRATION_METHOD = "Euler";
INTEGRATION_METHOD = "VI";

%JUMP_DIRECTION = "up";
JUMP_DIRECTION = "down";

res = 1; %Change resolution without changing timing
dt = .01/res; %timestep

ncf = 4; % # Contact Forces (Normal and Side Force for each foot)
nu = 4; % # Actuator Forces (Torque at each revolute joint)

%% Contact Phase Time Settings
% Jump with Flip
doubleContactTime = 0.5;
singleContactTime = 0.3;
flightTime = 1.0;

% %Jump Forward
% doubleContactTime = 0.4;
% singleContactTime = 0.2;
% flightTime = 0.4;

%%
T = doubleContactTime + singleContactTime + flightTime;
N = round(T/dt,0) + 1;

option.use_torque_jspeed_constraint= 1;

%% Environment Parameters
Kd = 1; % Damping Coefficent for Joint Friction
Kd_j = Kd*diag([0 0 0 1 1 1 1]); % Set which joints have damping  
option.mu = 0.8; %coefficient of friction

%% Robot Limits
option.q_joint_limit_hip = [-0.916; 4.18879]; % [Lower Bound, Upper Bound]
option.q_joint_limit_knee = [-2.697; -1.047]; % [Lower Bound, Upper Bound]

%Set torque limits for each phase 
%Each limit is formatted [Front Hip, Front Knee, Rear Hip, Rear Knee]
option.torque_saturation_stance = [33.5;33.5;33.5;33.5];
option.torque_saturation_swing = [10;10;33.5;33.5];
option.torque_saturation_flight = [15;15;15;15];
option.joint_vel_limit = 21;

%% torque and joint speed constraints
option.voltage_max =21.5;
option.current_max =60;
Kt=4/34; % from Unitree plot
torque_motor_max = 4;
speed_motor_max = 1700*2*pi/60; % max speed motor --> rad/s
gear_ratio =8.5; % joint_speed=speed_motr/gear_ratio
% eq: joint_speed=t2j_motor*x+b ; % rad/s
max_js= speed_motor_max; 
min_js=940*2*pi/60; % to compute the slope of line
alpha_motor= (min_js-max_js)/(torque_motor_max-0.2); % slope of the line
bm=(min_js-alpha_motor*torque_motor_max); % where bm=V/Kt=21.4/Kt=182 (rad/s)

%% Initial Conditions and Reference Angles
q_hip_i = 0.4*pi;
q_knee_i = -3*pi/4;

angles_init = [0; q_hip_i; q_knee_i; q_hip_i; q_knee_i]; % Trunk Angle, Hip Angle, Knee Angle
pFootTemp = pFootF([0; 0; angles_init]);

option.qi = [-pFootTemp([1,3]); angles_init]; % Calculate Initial Body XY from foot positon and joint angles
option.Qi = [option.qi; zeros([nx,1])]; % Start at rest

pFootFi = pFootF(option.qi);
pFootRi = pFootR(option.qi);

option.q_ref = angles_init(2:end);

%% Jump Distance Settings
option.platform_height = 3.0;
option.jump_length = -1.5;
option.flip = 3; % 0 = do not flip, otherwise is number of "backflips" (in direction of knee)

% option.platform_height = 0.0;
% option.jump_length = 1.0;
% option.flip = 0; % 0 = do not flip, otherwise is number of "backflips" (in direction of knee)
%%
if JUMP_DIRECTION == "up"
    option.qf = option.qi + [option.jump_length; option.platform_height; option.flip*(-2*pi); zeros([nx-3 1])];
elseif JUMP_DIRECTION =="down"
    option.qf = option.qi + [option.jump_length; -1*option.platform_height; option.flip*(-2*pi); zeros([nx-3 1])];
else
    error("JUMP_DIRECTION must be either up or down")
end

%% Contact schedule
% Set number of timesteps in each phase (double contact, single contact, flight)
double_contact_phase = round(doubleContactTime/dt);
single_contact_phase = round(singleContactTime/dt); 
flight_phase = N-double_contact_phase-single_contact_phase;

% build contact schedule (size N+1 to allow for proper contact constraint at k = N)
cs = [2*ones(1,double_contact_phase) ones(1,single_contact_phase) zeros(1,flight_phase) 0];

%% Initialize Optimization
opti = casadi.Opti();

%Create optimization variables
% ddq variables not needed for VI
if INTEGRATION_METHOD == "VI"
    X = opti.variable(2*nx+ncf+nu,N);
    dq  = X(1:nx,:);   % joint velocity
    q   = X(nx+1:2*nx,:);  % joint position
    f_c = X(2*nx+1:2*nx+ncf,:); % contact forces
    tau_joint = X(2*nx+ncf+1:end,:); % actuator torques
else
    X = opti.variable(3*nx+ncf+nu,N);
    ddq = X(1:nx,:);    % joint acceleration (includes floating base coordinates)
    dq  = X(nx+1:2*nx,:);   % joint velocity
    q   = X(2*nx+1:3*nx,:);  % joint position
    f_c = X(3*nx+1:3*nx+ncf,:); % contact forces
    tau_joint = X(3*nx+ncf+1:end,:); % actuator torques
end

Q = [q;dq]; % Q is q, q_dot for all timesteps
toc %display setup time
%%
disp_box('Building Cost Function');
tic; % Start Timing Cost Function Build

%Set weights
option.alpha = 1; % Weight for joint angle error from reference
option.alpha_f = 100; % Weight for terminal joint angle error
option.alpha_tau = .01; % Weight for control effort

qerr_f = q(4:end,N) - option.qf(4:end); % terminal joint angle error
J = option.alpha_f*(qerr_f')*qerr_f; % Add terminal cost

%Build Stage Cost
deleteStr = '';
for k=1:N-1
    %Display Step Number
    msg = ['Building Cost Step ' num2str(k) ' of ' num2str(N-1)];
    fprintf([deleteStr, msg]);
    deleteStr = repmat(sprintf('\b'), 1, length(msg));

    qerr = q(4:end,k) - [option.q_ref]; %joint error at timestep k
    tau_k = tau_joint(:,k); %control effort at timestep k
    J = J + option.alpha*(qerr')*qerr + option.alpha_tau * (tau_k')*tau_k; % Add stage cost for joint error and control effort
end

%Pass Cost Function to CASADI
opti.minimize(J);
fprintf('\nComplete!\n')
toc; %Output time for building cost function

%% Constraints
disp_box('Building Constraints');
tic;
deleteStr = '';
for k = 1:N-1
    %Display Step Number
    msg = ['Building Constraint ' num2str(k) ' of ' num2str(N-1) ' state: ' num2str(cs(k))];
    fprintf([deleteStr, msg]);
    deleteStr = repmat(sprintf('\b'), 1, length(msg));
    
    %Extract variables at timestep k
    qk = q(:,k); %Position States
    qjk = qk(4:end); %Position State of Joints
    dqk = dq(:,k); %Velocity or Momentum States
    dqjk = dqk(4:end); %Velocity or Momentum States for Joints 
    Qk = [qk;dqk];

    %Compute Position of Joints and Foot
    pFootFk = pFootF(qk); % Position of front foot contact
    pFootRk = pFootR(qk); % Position of rear foot contact
    pHipFk = pHipF(qk); % Position of front hip joint
    pKneeFk = pKneeF(qk); % Position of front knee joint
    pHipRk = pHipR(qk); % Position of rear hip joint
    pKneeRk = pKneeR(qk); % Position of rear knee joint

    %Compute Jacobians
    Jfk = Jf(qk); % Jacobian (combined front and rear contact jacobian)
    Jfk = Jfk([1,3,4,6], :); %Remove y components of Jacobian

    %Damping Torque
    tau_d = 1*Kd_j*Qk(nx+1:end);

%% Dynamics Constraints
    %% Euler
    if INTEGRATION_METHOD == "Euler"
        Mk = M(qk);    % Calculate Mass Matrix
        Ck = C(Qk); % Calculate Coriolis and Bias Torques
        if(cs(k+1) == 0)% if in flight, contact forces are 0, constraints are just ddq, M
            Ak = Mk;
            xk = ddq(:,k);
        elseif(cs(k+1) == 1) 
            %If in single contact, add effect of contact forces on rear foot, front contact forces are 0
            Ak = [Mk, Jfk(3:4,:).'];
            xk = [ddq(:,k);f_c(3:4,k)];
        else
            %if in double contact, add effects of forces at both contact points
            Ak = [Mk, Jfk.'];
            xk = [ddq(:,k);f_c(:,k)];
        end

        % Add damping forces and exogenous (e.g. actuator) forces to RHS
        % Note that body coordinates (x,y,theta) are not actuated
        bk =-Ck - tau_d + [0;0;0;tau_joint(:,k)];

        %Euler Dynamics Constraint
        opti.subject_to(Ak * xk == bk);

        % Euler timestep of dq,q
        dq_int = dq(:,k) + dt * ddq(:,k);
        q_int  = q(:,k)  + dt * dq_int;

        % integrated Q
        Q_int = [q_int;dq_int];

        % integration constraint
        Q_next = [q(:,k+1);dq(:,k+1)];
        opti.subject_to(Q_next == Q_int);
    end
    %% VI
    if INTEGRATION_METHOD == "VI"
        %VI Requires states and Jacobian at timestep k+1
        qkp1 = q(:,k+1);
        dqkp1 = dq(:,k+1);
        Jfkp1 = Jf(qkp1); %Jacobian at k+1     
        Jfkp1 = Jfkp1([1,3,4,6], :); %Remove y components of Jacobian

        %Define Midpoint Variables
        q_mp = (qk+qkp1)/2;
        dq_mp = (qkp1-qk)/dt;
        Q_mp = [q_mp;dq_mp];

        tau_d = 1*Kd_j*dq_mp; % Damping is now defined by midpoint velocity

        % Define left and right discrete forces
        % Note that body coordinates (x,y,theta) are not actuated
        f_d0 = (-tau_d+[0;0;0;tau_joint(:,k)] + (Jfk.')*(-1*f_c(:,k)));
        f_d1 = (-tau_d+[0;0;0;tau_joint(:,k+1)] + (Jfkp1.')*(-1*f_c(:,k+1)));
        
        % Define Implicit VI Update Rules
        p0VI = p0(Q_mp,dt) - 0.5*dt*f_d0;
        p1VI = p1(Q_mp,dt) + 0.5*dt*f_d1;
        opti.subject_to(dqk==p0VI);
        opti.subject_to(dqkp1==p1VI);
    end

%% Contact Constraints
    if cs(k+1) == 0 % If in flight
        opti.subject_to(f_c(:,k) == zeros(ncf,1)); % ground reaction forces must be zero
    elseif cs(k+1) == 1 
        % If single contact, rear foot is on the ground, front foot reactions are zero
        opti.subject_to(f_c(1:2,k) == zeros(ncf/2,1));
        opti.subject_to(f_c(4,k) <= -50); % Choose some minimum value of normal force to prevent slip
    else % If on the ground both forces are non-zero
        opti.subject_to(f_c(2,k) <= -50); % minimum value, as above
        opti.subject_to(f_c(4,k) <= -30); % minimum value, as above
    end

    contactRollCorrectionF = robotParams.rFoot * ((qk(3) + qk(4) + qk(5)) - (option.qi(3) + option.qi(4) + option.qi(5)));
    contactRollCorrectionR = robotParams.rFoot * ((qk(3) + qk(6) + qk(7)) - (option.qi(3) + option.qi(6) + option.qi(7)));

    if cs(k) == 1
        opti.subject_to((pFootRk - pFootRi) == [contactRollCorrectionR;0;0]); % rear foot contact must be on the ground
        opti.subject_to((pFootFk(3) - pFootFi(3)) >= 0.005); %front foot contact must be above the ground
    elseif cs(k) == 2
        opti.subject_to((pFootFk - pFootFi) == [contactRollCorrectionF;0;0]); % front foot contact must be on the ground
        opti.subject_to((pFootRk - pFootRi) == [contactRollCorrectionR;0;0]); % rear foot contact must be on the ground
    end

    % friction cone constraints
    opti.subject_to(f_c(1,k) <= -option.mu*f_c(2,k));
    opti.subject_to(f_c(1,k) >= option.mu*f_c(2,k));
    opti.subject_to(f_c(3,k) <= -option.mu*f_c(4,k));
    opti.subject_to(f_c(3,k) >= option.mu*f_c(4,k));
%% Motion and Kinematic Constraints
    if cs(k) ~= 0 % If in contact with the ground, keep joint locations above the ground
        opti.subject_to(pHipFk(3) - pFootRi(3) >= 0.07); % Front hip joint
        opti.subject_to(pKneeFk(3) - pFootRi(3) >= 0.05); % Front knee joint 
        opti.subject_to(pHipRk(3) - pFootRi(3) >= 0.07); % Rear hip joint
        opti.subject_to(pKneeRk(3) - pFootRi(3) >= 0.05); % Rear knee joint
    end

    %Joint Angle Limits
    qj_min = repmat([option.q_joint_limit_hip(1); option.q_joint_limit_knee(1)], [2,1]); %Joint Minima
    qj_max = repmat([option.q_joint_limit_hip(2); option.q_joint_limit_knee(2)], [2,1]); %Joint Maxima
    opti.subject_to(qjk <= qj_max);
    opti.subject_to(qjk >= qj_min);

    %Joint Velocity Limits OR Motor Dynamic Constraints
    if INTEGRATION_METHOD == "VI"
            if option.use_torque_jspeed_constraint
                %Estimate Voltage of the motor
                voltage = (0.5*tau_joint(:,k)*alpha_motor/gear_ratio +dq_mp(4:end)*gear_ratio)*Kt;
                opti.subject_to(-[1;1;1;1]*option.voltage_max <= voltage<=[1;1;1;1]*option.voltage_max);
            else
                %Limit the midpoint velocity
                opti.subject_to(dq_mp(4:end) <= ones([nx-3 1])*option.joint_vel_limit);
                opti.subject_to(dq_mp(4:end) >= -1*ones([nx-3 1])*option.joint_vel_limit);
            end
    else
            if option.use_torque_jspeed_constraint
                %Estimate Voltage of the motor
                voltage = (0.5*tau_joint(:,k)*alpha_motor/gear_ratio +dqjk*gear_ratio)*Kt;
                opti.subject_to(-[1;1;1;1]*option.voltage_max <= voltage<=[1;1;1;1]*option.voltage_max);
            else
                %Limit the joint velocity
                opti.subject_to(dqjk <= ones([nx-3 1])*option.joint_vel_limit);
                opti.subject_to(dqjk >= -1*ones([nx-3 1])*option.joint_vel_limit);
            end
    end
%% Control Effort Limits

    %Limits are set to 2x the value to account for having both a left and 
    % right actuator in the full 3D robot
    if(cs(k) == 2)
        %Contact Phase
        opti.subject_to(tau_joint(:,k) <= 2*option.torque_saturation_stance);
        opti.subject_to(tau_joint(:,k) >= -2*option.torque_saturation_stance);
    elseif(cs(k) == 1)
        opti.subject_to(tau_joint(:,k) <= 2*option.torque_saturation_swing);
        opti.subject_to(tau_joint(:,k) >= -2*option.torque_saturation_swing);
    else
        %Flight Phase
        opti.subject_to(tau_joint(:,k) <= 2*option.torque_saturation_flight);
        opti.subject_to(tau_joint(:,k) >= -2*option.torque_saturation_flight);
    end

end

%% Initial/Terminal Constraints
%Initial Conditions
opti.subject_to(q(:,1) == option.qi);    % inital configuration
opti.subject_to(dq(:,1) == zeros(nx,1)); % initial velocity

%Terminal Conditions
opti.subject_to(f_c(:,N) == zeros([ncf 1])); %Contact Forces are 0 at the end
opti.subject_to(tau_joint(:,N) == zeros([nu 1])); % Joints Torques are 0 at the end

%Terminal constraint for Body coordinates: y and theta must match exactly
%X coordinate is allowed to exceed the target (a further jump) if one is found
opti.subject_to(q(1,N) <= option.qf(1)); 
opti.subject_to(q(2,N) == option.qf(2));
opti.subject_to(q(3,N) == option.qf(3));

%Terminal constraint for Joint coordinates
opti.subject_to(q(4:7,N) == option.qf(4:end));

%Momentum Constraints if VI, else Velocity Constraints
%Goal is to set joint velocities to 0. For VI, this corresponds to non-zero
%momenta, because body coordinate velocities are allowed to be non-zero.
if INTEGRATION_METHOD == "VI"
    opti.subject_to(dq(4:end,N) == VI_JointTermCon([q(:,N);dq(1:3,N)]))
else
    opti.subject_to(dq(4:end,N) == zeros(nx-3,1))
end
fprintf('\nComplete!\n')
toc;

%% Set Initial Guess
posGuess = repmat(option.qi,1,N); %Guess is initial state for all time steps
opti.set_initial(q,posGuess);
opti.set_initial(tau_joint,repmat(2*ones(nu,1),1,N)); % Guess is constant torque

%% Solve
disp_box('Starting IPOPT');
p_opts = struct('expand',true);
s_opts = struct('max_iter',2000);
opti.solver('ipopt',p_opts,s_opts);

%Time Solution
tic;
sol = opti.solve();
toc;

%% Plot

%Extract position, velocity, control effort, and contact forces
pos = opti.debug.value(q);
vel = opti.debug.value(dq);
Q_TO = [pos;vel];
control = opti.debug.value(tau_joint);
contactForces = opti.debug.value(f_c);

%% Plot Generalized Coordinates vs. Time
figure(1)
clf()
hold on
for k = 1:nx
    plot((0:N-1)*dt, pos(k,:))
end
hold off
legend(["x", "y", "th_b", "q_{hipF}","q_{kneeF}", "q_{hipR}", "q_{kneeR}"], "location", "southwest")
xlabel("Time (s)")
ylabel("Generalized Coordinate")
%%
figure(2)
clf()
hold on
for k = 1:nu
    plot(control(k,:))
end
hold off
%% Plot the XY trajectory of each robot node, and Trunk CG
for k = 1:N
    footFPos(:,k) = pFootF(pos(:,k));
    footRPos(:,k) = pFootR(pos(:,k));
    hipFPos(:,k) = pHipF(pos(:,k));
    kneeFPos(:,k) = pKneeF(pos(:,k));
    hipRPos(:,k) = pHipR(pos(:,k));
    kneeRPos(:,k) = pKneeR(pos(:,k));
end

figure(3)
clf()
hold on
plot(pos(1,:), pos(2,:))
plot(footFPos(1,:), footFPos(3,:))
plot(footRPos(1,:), footRPos(3,:))
plot(hipFPos(1,:), hipFPos(3,:))
plot(kneeFPos(1,:), kneeFPos(3,:))
plot(hipRPos(1,:), hipRPos(3,:))
plot(kneeRPos(1,:), kneeRPos(3,:))
hold off
axis equal
xlabel("X (m)")
ylabel("Y(m)")
%% Plot Contact Forces
figure();
clf();
hold on
for ii = 1:ncf
    plot(contactForces(ii,:));
end
%%
animate_Quad2D(pos);
%% Simulink Simulation

%Check which version of Simulink is installed
verStruct = ver;

idx_SL = find(ismember({verStruct.Name}, 'Simulink'));

cd(fullfile('..','SimulinkModels','Quadruped2DSimulink'))

if isempty(idx_SL)
    error('Simulink not detected on machine.\nPlease install Simulink (2023b or newer) to run simulation of TO result', 0)
else
    SL_Version = str2double(verStruct(idx_SL).Version);
    SL_Release = verStruct(idx_SL).Release;
    if SL_Version >= 23.2
        out = sim(fullfile('.','Quadruped2D_Simulink.slx'), "StopTime", num2str((N-1)*dt));
    else
        error('Installed Simulink version %s is not currently supported.\nPlease install Simulink 2023b or newer, or contact author for support', SL_Release);
    end
end

cd(parentFile);

%% Simulink Plotting

tTO = (0:max(size(pos))-1)*dt;

if INTEGRATION_METHOD == "Euler"
    titles = ["Body X - Euler", "Body Y - Euler", "Body Theta - Euler"];
end

if INTEGRATION_METHOD == "VI"
    titles = ["Body X - VI", "Body Y - VI", "Body Theta - VI"];
end
yLabels = ["X (m)", "Y (m)", "Theta (rad)"];

for ii = 1:3
    figure()
    clf()
    hold on
    plot(out.tout, squeeze(out.qOut(:,ii)), '-b', 'LineWidth', 1.5);
    plot(tTO, pos(ii,:), '--r', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel(yLabels(ii));
    title(titles(ii));
    legend("Simulated", "TO Reference", 'Location','northwest')
end
