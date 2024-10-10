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

%Set Current Folder to Folder of Main Function
cd(parentFile);

%% Robot Parameters and Dynamics Generation 
disp_box('Generating Dynamics')

[nx, robotStruct, robotParams] = getSingleAcroStruct();
genDynamics(nx, robotStruct);

%% Integration Settings
disp_box('Initializing Trajectory Optimization')
tic %start timing setup time

%Choose Integration Method (Euler or VI)
INTEGRATION_METHOD = "Euler";
INTEGRATION_METHOD = "VI";

res = 1; %Change resolution without changing timing
dt = .01/res; %timestep

ncf = 2; % # Contact Forces (x and z forces at the pin joint)
nu = 1; % # Actuator Forces (Torque at the single revolute joint)

%% Contact Phase Time Settings
% Jump with Flip
pinTime = 0.8;
flightTime = 0.9;

T = pinTime + flightTime;
N = round(T/dt,0) + 1;

%% Contact schedule
% Set number of timesteps in each phase (pin, flight)
pin_phase = round(pinTime/dt);
flight_phase = N-pin_phase;

% build contact schedule (size N+1 to allow for proper contact constraint at k = N)
cs = [1*ones(1,pin_phase) zeros(1,flight_phase) 0];

%% Environment Parameters
Kd = 0; % Damping Coefficent for Joint Friction
Kd_j = Kd*diag([0 0 1]); % Set which joints have damping (only revolute joint)

%% Initial Conditions 
option.qi = [0;0;0]; % Start at downward position at origin
option.dqi = [0;0;0];  % Start at rest

pPini = pPin(option.qi);

%% Terminal Conditions
option.pinHeight = 2.0;
option.jumpLength = 2.5;
option.flip = -1;

option.qf = [option.jumpLength; -option.pinHeight; option.flip*2*pi]; % End having jumped some distance, and flipped some amount.
option.dqf = [0;0;0]; % end at rest 

%% Robot Limits
%Set torque limits for each phase 
%Each limit is formatted [Pin Joint Actuator, Elbow Actuator]
option.torque_saturation_pin = 20;
option.torque_saturation_flight = 0;

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
option.alpha_f = 1; % Weight for terminal joint angle error
option.alpha_tau = 0.1; % Weight for control effort

qerr_f = q(:,N) - option.qf; % terminal joint angle error
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
    msg = ['Building Constraint ' num2str(k) ' of ' num2str(N-1)];
    fprintf([deleteStr, msg]);
    deleteStr = repmat(sprintf('\b'), 1, length(msg));
    
    %Extract variables at timestep k
    qk = q(:,k); %Position States
    qjk = q(3:end); %Get just joint position
    dqk = dq(:,k); %Velocity or Momentum States
    dqjk = dqk(3:end); %Get just joint velocity
    Qk = [qk;dqk];

    %Compute Position of Pin Contact
    pPink = pPin(qk);

    %Compute Jacobian;
    Jfk = Jf(qk); %Jacobian
    Jfk = Jfk([1,3], :); %Remove y components of Jacobian

%% Dynamics Constraints
    %% Euler
    if INTEGRATION_METHOD == "Euler"
        Mk = M(qk);    % Calculate Mass Matrix
        Ck = C(Qk); % Calculate Coriolis and Bias Torques

        if (cs(k+1) == 0) %if in flight, contact forces are 0, constraints are just ddq, M
            Kd_j = zeros(nx); % If in flight, no damping
            Ak = Mk;
            xk = ddq(:,k);
        else
            %if in contact, add effects of forces at contact point
            Ak = [Mk, Jfk.'];
            xk = [ddq(:,k);f_c(:,k)];
        end

        tau_d = Kd_j*Qk(nx+1:end);

        % Add damping forces and exogenous (e.g. actuator) forces to RHS
        % Note that body coordinates (x,y) are not actuated, and theta is
        % not actuated when in the flight phase
        bk =-Ck - tau_d + [0;0;tau_joint(:,k)];

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

        if cs(k+1) == 0
            Kd_j = zeros(nx);
        end

        tau_d = Kd_j*dq_mp; % Damping is now defined by midpoint velocity

        % Define left and right discrete forces
        % Note that body coordinates (x,y) are not actuated, and theta is
        % not actuated when in the flight phase

        f_d0 = (-tau_d+[0;0;tau_joint(:,k)] + (Jfk.')*(-1*f_c(:,k)));
        f_d1 = (-tau_d+[0;0;tau_joint(:,k+1)] + (Jfkp1.')*(-1*f_c(:,k+1)));

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

    if cs(k) == 1
        opti.subject_to((pPink - pPini) == [0;0;0]); % pin must be at original position
    end

    %% Control Effort Limits

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
opti.subject_to(q(:,1) == option.qi);    % inital position
opti.subject_to(dq(:,1) == option.dqi); % initial velocity

%Terminal Conditions
opti.subject_to(q(:,N) == option.qf); % final position
opti.subject_to(tau_joint(:,N) == zeros([nu 1])); % Joints Torques are 0 at the end

fprintf('\nComplete!\n')
toc;

%% Set Initial Guess
posGuess = repmat(option.qi,1,N); %Guess is initial state for all time steps
opti.set_initial(q,posGuess);
opti.set_initial(tau_joint,repmat(0.1*ones(nu,1),1,N)); % Guess is constant torque

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
legend(["X", "Y", "\theta"], "location", "southwest")
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

%% Plot Contact Forces
figure();
clf();
hold on
for ii = 1:ncf
    plot(contactForces(ii,:));
end

%%
animate_SingleAcro(pos);
%% Simulink Simulation

%Check which version of Simulink is installed
verStruct = ver;

idx_SL = find(ismember({verStruct.Name}, 'Simulink'));

cd(fullfile('..','SimulinkModels','SingleAcroSimulink'))

if isempty(idx_SL)
    error('Simulink not detected on machine.\nPlease install Simulink (2023b or newer) to run simulation of TO result', 0)
else
    SL_Version = str2double(verStruct(idx_SL).Version);
    SL_Release = verStruct(idx_SL).Release;
    if SL_Version >= 23.2
        out = sim(fullfile('.','SingleAcroSimulink.slx'), "StopTime", num2str((N-1)*dt));
    else
        error('Installed Simulink version %s is not currently supported.\nPlease install Simulink 2023b or newer, or contact author for support', SL_Release);
    end
end

cd(parentFile);

%% Simulink Plotting (Currently Broken)

tTO = (0:max(size(pos))-1)*dt;

if INTEGRATION_METHOD == "Euler"
    titles = ["Pivot X - Euler", "Pivot Y - Euler", "Theta - Euler"];
end

if INTEGRATION_METHOD == "VI"
    titles = ["Pivot X - VI", "Pivot Y - VI", "Theta - VI"];
end
yLabels = ["Pivot X (m)", "Pivot Y (m)", "Theta (rad)"];

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

