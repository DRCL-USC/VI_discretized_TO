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

[nx, robotStruct, robotParams] = getDoubleAcroStruct();
genDynamics(nx, robotStruct); %Generate Dynamics!

%% Integration Settings
disp_box('Initializing Trajectory Optimization')
tic %start timing setup time

%Choose Integration Method (Euler or VI)
INTEGRATION_METHOD = "Euler";
INTEGRATION_METHOD = "VI";

res = 1; %Change resolution without changing timing
dt = .01/res; %timestep

ncf = 2; % # Contact Forces (Normal and Side Force for each foot)
nu = 2; % # Actuator Forces (Torque at each revolute joint)

%% Contact Phase Time Settings
% Jump with Flip
pinTime = 1.5;
flightTime = 1.5;

%%
T = pinTime + flightTime;
N = round(T/dt,0) + 1;

% %% Environment Parameters
% Kd = 0; % Damping Coefficent for Joint Friction
% Kd_j = Kd*diag([0 0 0 1]); % Set which joints have damping  

%% Robot Limits
%Set torque limits for each phase 
%Each limit is formatted [Pin Joint Actuator, Elbow Actuator]
option.torque_saturation_pin = [33.5;33.5];
option.torque_saturation_flight = [0;33.5];
option.joint_vel_limit = 21;

%% Initial Conditions and Reference Angles

option.qi = [0;0;0;0]; %start in downward position at the origin
option.Qi = [option.qi; zeros([nx,1])]; % Start at rest

pPini = pPin(option.qi);

%% Jump Distance Settings
option.pinHeight = 3.0;
option.jumpLength = 2.0;
option.flip = 0; % 0 = do not flip, otherwise is number of "backflips" (in direction of knee)

%% Terminal Conditions

hPinF = -1*option.pinHeight + robotParams.L1 + robotParams.L2;

option.qf = option.qi + [option.jumpLength; hPinF; option.flip*(-2*pi); 0];

%% Contact schedule
% Set number of timesteps in each phase (double contact, single contact, flight)
pin_phase = round(pinTime/dt);
flight_phase = N-pin_phase;

% build contact schedule (size N+1 to allow for proper contact constraint at k = N)
cs = [1*ones(1,pin_phase) zeros(1,flight_phase) 0];

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

qerr_f = q(:,N) - option.qf; % terminal error
J = option.alpha_f*(qerr_f')*qerr_f; % Add terminal cost

%Build Stage Cost
deleteStr = '';
for k=1:N-1
    %Display Step Number
    msg = ['Building Cost Step ' num2str(k) ' of ' num2str(N-1)];
    fprintf([deleteStr, msg]);
    deleteStr = repmat(sprintf('\b'), 1, length(msg));

    tau_k = tau_joint(:,k); %control effort at timestep k
    J = J + option.alpha_tau * (tau_k')*tau_k; % Add stage cost for control effort
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
    pPink = pPin(qk); % Position of front foot contact

    %Compute Jacobians
    Jfk = Jf(qk); % Jacobian (combined front and rear contact jacobian)
    Jfk = Jfk([1,3], :); %Remove y components of Jacobian

%% Dynamics Constraints
    %% Euler
    if INTEGRATION_METHOD == "Euler"
        Mk = M(qk);    % Calculate Mass Matrix
        Ck = C(Qk); % Calculate Coriolis and Bias Torques
        if(cs(k+1) == 0)% if in flight, contact forces are 0, constraints are just ddq, M
            Ak = Mk;
            xk = ddq(:,k);
        else
            %if in contact, add effects of forces at contact points
            Ak = [Mk, Jfk.'];
            xk = [ddq(:,k);f_c(:,k)];
        end

        % Add damping forces and exogenous (e.g. actuator) forces to RHS
        % Note that body coordinates (x,y) are not actuated
        bk =-Ck + [0;0;tau_joint(:,k)];

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
        Jfkp1 = Jfkp1([1,3], :); %Remove y components of Jacobian

        %Define Midpoint Variables
        q_mp = (qk+qkp1)/2;
        dq_mp = (qkp1-qk)/dt;
        Q_mp = [q_mp;dq_mp];

        % Define left and right discrete forces
        % Note that body coordinates (x,y) are not actuated
        f_d0 = ([0;0;tau_joint(:,k)] + (Jfk.')*(-1*f_c(:,k)));
        f_d1 = ([0;0;tau_joint(:,k+1)] + (Jfkp1.')*(-1*f_c(:,k+1)));
        
        % Define Implicit VI Update Rules
        p0VI = p0(Q_mp,dt) - 0.5*dt*f_d0;
        p1VI = p1(Q_mp,dt) + 0.5*dt*f_d1;
        opti.subject_to(dqk==p0VI);
        opti.subject_to(dqkp1==p1VI);
    end

%% Contact Constraints
    if cs(k+1) == 0 % If in flight
        opti.subject_to(f_c(:,k) == zeros(ncf,1)); % ground reaction forces must be zero
    end

    if cs(k) == 1 % If in contact
        opti.subject_to((pPink - pPini) == [0;0;0]); % pin point must not move
    end

%% Motion and Kinematic Constraints

    %Joint Velocity Limits OR Motor Dynamic Constraints
    if INTEGRATION_METHOD == "VI"
        %Limit the midpoint velocity
        opti.subject_to(dq_mp(4:end) <= ones([nx-3 1])*option.joint_vel_limit);
        opti.subject_to(dq_mp(4:end) >= -1*ones([nx-3 1])*option.joint_vel_limit);
    else
        %Limit the joint velocity
        opti.subject_to(dqjk <= ones([nx-3 1])*option.joint_vel_limit);
        opti.subject_to(dqjk >= -1*ones([nx-3 1])*option.joint_vel_limit);
    end
%% Control Effort Limits

    %Limits are set to 2x the value to account for having both a left and 
    % right actuator in the full 3D robot
    if(cs(k) == 1)
        %Contact Phase
        opti.subject_to(tau_joint(:,k) <= option.torque_saturation_pin);
        opti.subject_to(tau_joint(:,k) >= -1*option.torque_saturation_pin);
    else
        %Flight Phase
        opti.subject_to(tau_joint(:,k) <= option.torque_saturation_flight);
        opti.subject_to(tau_joint(:,k) >= -1*option.torque_saturation_flight);
    end

end

%% Initial/Terminal Constraints
%Initial Conditions
opti.subject_to(q(:,1) == option.qi);    % inital configuration
opti.subject_to(dq(:,1) == zeros(nx,1)); % initial velocity

%Terminal Conditions
opti.subject_to(f_c(:,N) == zeros([ncf 1])); %Contact Forces are 0 at the end
opti.subject_to(tau_joint(:,N) == zeros([nu 1])); % Joints Torques are 0 at the end

%Terminal constraint for generalized coordinates
opti.subject_to(q(:,N) == option.qf);

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
legend(["x", "y", "\theta_{Pivot}", "\theta_{Elbow}"], "location", "southwest")
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
    pinPos(:,k) = pPin(pos(:,k));
    elbowPos(:,k) = pElbow(pos(:,k));
    tipPos(:,k) = pTip(pos(:,k));
end

figure(3)
clf()
hold on
plot(pinPos(1,:), pinPos(3,:))
plot(elbowPos(1,:), elbowPos(3,:))
plot(tipPos(1,:), tipPos(3,:))
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
animate_DoubleAcro(pos);
%% Simulink Simulation

%Check which version of Simulink is installed
verStruct = ver;

idx_SL = find(ismember({verStruct.Name}, 'Simulink'));

cd(fullfile('..','SimulinkModels','DoubleAcroSimulink'))

if isempty(idx_SL)
    error('Simulink not detected on machine.\nPlease install Simulink (2023b or newer) to run simulation of TO result', 0)
else
    SL_Version = str2double(verStruct(idx_SL).Version);
    SL_Release = verStruct(idx_SL).Release;
    if SL_Version >= 23.2
        out = sim(fullfile('.','DoubleAcroSimulink.slx'), "StopTime", num2str((N-1)*dt));
    else
        error('Installed Simulink version %s is not currently supported.\nPlease install Simulink 2023b or newer, or contact author for support', SL_Release);
    end
end

cd(parentFile);

%% Simulink Plotting

tTO = (0:max(size(pos))-1)*dt;

if INTEGRATION_METHOD == "Euler"
    titles = ["Body X - Euler", "Body Y - Euler", "Pivot Angle - Euler", "Elbow Angle - Euler"];
end

if INTEGRATION_METHOD == "VI"
    titles = ["Body X - VI", "Body Y - VI", "Pivot Angle - VI", "Elbow Angle"];
end
yLabels = ["X (m)", "Y (m)", "Pivot Angle (rad)", "Elbow Angle (rad)"];


for ii = 1:nx
    figure()
    clf()
    hold on
    plot(out.qOut.time, out.qOut.signals.values(:,ii), '-b', 'LineWidth', 1.5);
    plot(tTO, pos(ii,:), '--r', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel(yLabels(ii));
    title(titles(ii));
    legend("Simulated", "TO Reference", 'Location','northwest')
end
