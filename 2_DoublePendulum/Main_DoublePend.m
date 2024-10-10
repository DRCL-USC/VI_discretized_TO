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

[nx, robotStruct, robotParams] = getDoublePendStruct();
genDynamics(nx, robotStruct);

%% Integration Settings
disp_box('Initializing Trajectory Optimization')
tic %start timing setup time

%Choose Integration Method (Euler or VI)
INTEGRATION_METHOD = "Euler";
INTEGRATION_METHOD = "VI";

res = 1; %Change resolution without changing timing
dt = .01/res; %timestep

T = 2.5;
N = round(T/dt,0) + 1;

ncf = 0; % # Contact Forces (None)
nu = 2; % # Actuator Forces (Torque at the single revolute joint)

%% Environment Parameters
Kd = 0; % Damping Coefficent for Joint Friction
Kd_j = Kd * diag([1 1]); % Select which joints experience damping

%% Initial Conditions 
option.qi = [0; 0]; % Start at downward position
option.dqi = zeros([nx,1]);  % Start at rest

%% Terminal Conditions
option.qf = [pi; 0]; % End in upward position
option.dqf = zeros([nx,1]); % end at rest 

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
option.alpha_f = 100; % Weight for terminal joint angle error
option.alpha_ref = 1; % Weight for error to final state (encourage early arrival) 
option.alpha_tau = 10; % Weight for control effort

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

    qerr = q(:,N) - option.qf;
    J = J + option.alpha_ref * qerr.'*qerr; % Add stage cost for error to final state (encourage early arrival)
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
    dqk = dq(:,k); %Velocity or Momentum States
    Qk = [qk;dqk];

%% Dynamics Constraints
    %% Euler
    if INTEGRATION_METHOD == "Euler"
        Mk = M(qk);    % Calculate Mass Matrix
        Ck = C(Qk); % Calculate Coriolis and Bias Torques
        xk = ddq(:,k);

        % Damping Torque
        tau_d = Kd_j*Qk(nx+1:end);

        % Add damping forces and exogenous (e.g. actuator) forces to RHS
        bk =-Ck - tau_d + tau_joint(:,k);

        %Euler Dynamics Constraint
        opti.subject_to(Mk * xk == bk);

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

        %Define Midpoint Variables
        q_mp = (qk+qkp1)/2;
        dq_mp = (qkp1-qk)/dt;
        Q_mp = [q_mp;dq_mp];

        tau_d = Kd_j*dq_mp; % Damping is now defined by midpoint velocity

        % Define left and right discrete forces
        f_d0 = (-tau_d+tau_joint(:,k));
        f_d1 = (-tau_d+tau_joint(:,k));
        
        % Define Implicit VI Update Rules
        p0VI = p0(Q_mp,dt) - 0.5*dt*f_d0;
        p1VI = p1(Q_mp,dt) + 0.5*dt*f_d1;
        opti.subject_to(dqk==p0VI);
        opti.subject_to(dqkp1==p1VI);
    end

end

%% Initial/Terminal Constraints
%Initial Conditions
opti.subject_to(q(:,1) == option.qi);    % inital position
opti.subject_to(dq(:,1) == option.dqi); % initial velocity

%Terminal Conditions
opti.subject_to(q(:,N) == option.qf); % final position
opti.subject_to(dq(:,N) == option.dqf); % final velocity
opti.subject_to(tau_joint(:,N) == zeros([nu 1])); % Joints Torques are 0 at the end

fprintf('\nComplete!\n')
toc;

%% Set Initial Guess
posGuess = repmat(option.qi,1,N) + (repmat(option.qf-option.qi,1,N).*linspace(0,1,N));

%posGuess = repmat(option.qi,1,N); %Guess is initial state for all time steps
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

%% Plot Generalized Coordinates vs. Time
figure(1)
clf()
hold on
for k = 1:nx
    plot((0:N-1)*dt, pos(k,:))
end
hold off
legend(["\theta_1", "\theta_2"], "location", "southeast")
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

%%
animate_DoublePend(pos);
%% Simulink Simulation

%Check which version of Simulink is installed
verStruct = ver;

idx_SL = find(ismember({verStruct.Name}, 'Simulink'));

cd(fullfile('..','SimulinkModels','DoublePendSimulink'))

if isempty(idx_SL)
    error('Simulink not detected on machine.\nPlease install Simulink (2023b or newer) to run simulation of TO result', 0)
else
    SL_Version = str2double(verStruct(idx_SL).Version);
    SL_Release = verStruct(idx_SL).Release;
    if SL_Version >= 23.2
        out = sim(fullfile('.','DoublePendSimulink.slx'), "StopTime", num2str((N-1)*dt));
    else
        error('Installed Simulink version %s is not currently supported.\nPlease install Simulink 2023b or newer, or contact author for support', SL_Release);
    end
end

cd(parentFile);

%% Simulink Plotting (Currently Broken)

tTO = (0:max(size(pos))-1)*dt;

if INTEGRATION_METHOD == "Euler"
    titles = ["Pivot Angle - Euler", "Elbow Angle - Euler"];
end

if INTEGRATION_METHOD == "VI"
    titles = ["Pivot Angle - VI", "Elbow Angle - VI"];
end
yLabels = ["Pivot Angle (rad)", "Elbow Angle (rad)"];

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

