clc; close all; clear
Initiate_Simulation;
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

currentPATH = getenv("PATH");
MA27PATH = fullfile(pwd, 'CoinHSL-archive.v2023.11.17.x86_64-w64-mingw32-libgfortran5', 'bin');
newPATH = [currentPATH, ';', MA27PATH];
setenv("PATH", newPATH);

%Set Current Folder to Folder of Main Function
cd(parentFile);

%% Robot Parameters and Dynamics Generation (Adapted (with assumptions) from HECTOR model)
% Ignoring Linkages, Linkage Plate, etc. No Arms
disp_box('Generating Dynamics')

%% Generate Composite Trunk CG and Moment of Inertia
%Includes Hip Yaw and Roll Motors and Mounting Brackets

%Hector Hip Joint Transforms
TrunkRToHip1R = [0; -0.1265];
Hip1RToHip2R = [0.053-0.014; -0.071];
Motor1RToHip2R = [-0.014; 0];
Hip2RToMotor2R = [-0.06; 0];

%Hip 1 Params
CG_Hip1 = [0.0268; -0.0272];
mHip1 = 2*0.173+.00000011;
IHip1 = 2*0.00024;
HipJointToHip1CG = CG_Hip1 + -1*(Hip1RToHip2R + Hip2RToMotor2R);

%Hip 2 Params
CG_Hip2 = [-0.0308; 0];
mHip2 = 2*0.0722;
IHip2 = 2*0.000120;
HipJointToHip2CG = CG_Hip2 + -1*Hip2RToMotor2R;

%Motor 1 Params (Hip Yaw)
CG_Motor1 = [0.02; 0];
mMotor1 = 2*0.605;
IMotor1 = 2*0.00048;
HipJointToMotor1CG = CG_Motor1 + -1*(Motor1RToHip2R + Hip2RToMotor2R);

%Motor 2 Params (Hip Roll)
CG_Motor2 = [0; 0];
mMotor2 = 2*0.605;
IMotor2 = 2*0.00071;
HipJointToMotor2CG = CG_Motor2;

%Body Params
CG_Body = [0; 0];
mBody = 5.75;
IBody = 53149*1e-6;
HipJointToBodyCG = CG_Body + -1*(TrunkRToHip1R + Hip1RToHip2R + Hip2RToMotor2R);

% Calculate Composite Trunk CG
trunkMasses = [mHip1; mHip2; mMotor1; mMotor2; mBody];
trunkComponentCGs = [HipJointToHip1CG HipJointToHip2CG HipJointToMotor1CG HipJointToMotor2CG HipJointToBodyCG].';

trunkCG = sum((trunkMasses*ones(1,2)).*trunkComponentCGs)/sum(trunkMasses);

%Calculate Composite Trunk Moment of Inertia
trunkMOIs = [IHip1; IHip2; IMotor1; IMotor2; IBody];

%Calculate distances between each component and the composite CG
trunkComponentsCGsToTrunkCG = trunkCG - trunkComponentCGs;
trunkComponentDistToCG = vecnorm(trunkComponentsCGsToTrunkCG.').';

ITrunk = sum(trunkMOIs + trunkMasses.*trunkComponentDistToCG.^2);

%% Dynamics Model Parameters
%Composite Trunk Body Params
robotParams.L1 = trunkCG(2)*2; %For Display Only
robotParams.CG_TrunkToHipJoint = -1*trunkCG.';
robotParams.m1 = sum(trunkMasses);
robotParams.I1 = ITrunk;

%Thigh Body Params
robotParams.L2 = 0.22;
robotParams.CG2 = [-0.00143; -0.06115];
robotParams.m2 = 2*0.397;
robotParams.I2 = 2*0.00218;

%Calf Body Params
robotParams.CG3 = [0.00165; -0.06438];
robotParams.L3 = 0.22;
robotParams.m3 = 2*0.136;
robotParams.I3 = 2*0.00072;

%Foot Body Params
robotParams.pfCG = [0.00239; -0.02261];
robotParams.m4 = 2*0.08425;
robotParams.I4 = 2*0.00011;

%Contact Points
robotParams.pfc1 = [0.09; -0.039];
robotParams.pfc2 = [-0.06; -0.039];

generateDynamics(robotParams);
oldpath = path;
%%
path(genpath('dynamicsFunctions'),  oldpath);

%% Integration Settings
disp_box('Initializing Trajectory Optimization')
tic %start timing setup time
INTEGRATION_METHOD = "Euler";
INTEGRATION_METHOD = "VI";

%JUMP_DIRECTION = "up";
JUMP_DIRECTION = "down";

res = 1; %Change resolution without changing timing
dt = .01/res; %timestep

nx = 6; % # of Joint Positions (3 DOF Trunk, 3 revolute joints)
ncf = 4; % # Contact Forces (Normal and Side Force for each foot)
nu = 3; % # Actuator Forces (Torque at each revolute joint)

%% Contact Phase Time Settings
%Jump with Flip
doubleContactTime = 0.65;
singleContactTime = 0.0;
flightTime = 0.75;
flightTime = 0.7;

%Jump with double Flip
% doubleContactTime = 0.75;
% singleContactTime = 0.0;
% flightTime = 0.9;

% %Jump Forward
% doubleContactTime = 0.5;
% singleContactTime = 0;
% flightTime = 0.4;

%%
%T = contactTime + flightTime;
T = doubleContactTime + singleContactTime + flightTime;
N = round(T/dt,0);

option.res = res;
option.N = N;

%% Environment Parameters
Kd = 1;
Kd_j = Kd*diag([0 0 0 1 1 1]);
option.mu = 0.8;

%% Robot Limits
option.q_joint_limit_hip = (pi/180)*[-120; 120];
option.q_joint_limit_knee = (pi/180)*[-160; -15];
option.q_joint_limit_ankle = (pi/180)*[-75; 75];

option.torque_saturation_stance= [33.5;51;33.5];
option.torque_saturation_flight= 1*[15;30;15];
option.joint_vel_limit = 21;

%% Initial Conditions and Reference Angles
angles_init = [0; pi/4; -pi/2; pi/4]; % Trunk Angle, Hip Angle, Knee Angle
pf1i = [0; 0]; %Foot Front Contact Position
option.qi = [xyInitFromAngles(pf1i, angles_init); angles_init]; % Calculate Initial Body XY from foot positon and joint angles
option.Qi = [option.qi; zeros([nx,1])]; % Start at rest

pf2i = pf2(option.qi);

option.q_ref = angles_init(2:end);

%% Jump Distance Settings
% option.platform_height = 1.0 + 1*rand(1);
% option.jump_length = -0.5 - 0.5*rand(1);
% 

option.flip = 1; % 0 = do not flip, otherwise is number of "backflips" (in direction of knee)
%% Flat Ground Flip Data Points

option.platform_height = 0.0;
% option.jump_length = -0.4;


option.jump_length = -0.362316774830248;

option.jump_length = -0.050794726517402;
% 
option.jump_length = -0.252943698490164;
% 
option.jump_length = -0.111399287546819;
% 
option.jump_length = -0.472444267599638;

%% Double Flip Data Points
% option.platform_height = 1.807506835434298;
% option.jump_length = -0.882444267599638;
% 
% option.platform_height =  1.421761282626275;
% option.jump_length = -0.957867762594534;
% % 
% option.platform_height = 1.678735154857773;
% option.jump_length =-0.878870065289167;
% 
% option.platform_height = 1.276922984960890
% option.jump_length = -0.523085695315577
% 
% option.platform_height = 1.694828622975817;
% option.jump_length = -0.658549740030430;

%% Single Flip Data Points
% option.platform_height = 1.452895968537810;
% option.jump_length = -0.438096044888052;

% option.platform_height = 1.1907792285465;
% option.jump_length = -1.1296550364447;

% option.platform_height = 1.485296390880308;
% option.jump_length = -0.687150084472884;

% option.platform_height = 1.479746213;	
% option.jump_length = -0.59672221;

% option.platform_height =  1.085593343905781;
% option.jump_length = -0.753023044009804;



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
double_contact_phase = round(doubleContactTime/dt);
single_contact_phase = round(singleContactTime/dt);
flight_phase = N-double_contact_phase-single_contact_phase; % timesteps spent in the air
% build contact schedule (it needs to be size N+1 because of constraints)
cs = [2*ones(1,double_contact_phase) ones(1,single_contact_phase) zeros(1,flight_phase) 0];

%% Initialize Optimization
opti = casadi.Opti();

%Create optimization variables
% ddq variables not needed for VI
if INTEGRATION_METHOD == "VI"
    X = opti.variable(2*nx+ncf+nu,N);
    dq  = X(1:nx,:);   % joint velocity
    q   = X(nx+1:2*nx,:);  % joint position
    f_r = X(2*nx+1:2*nx+ncf,:); % contact forces
    tau_joint = X(2*nx+ncf+1:end,:); % actuator torques
else
    X = opti.variable(3*nx+ncf+nu,N);
    ddq = X(1:nx,:);    % joint acceleration (includes floating base coordinates)
    dq  = X(nx+1:2*nx,:);   % joint velocity
    q   = X(2*nx+1:3*nx,:);  % joint position
    f_r = X(3*nx+1:3*nx+ncf,:); % contact forces
    tau_joint = X(3*nx+ncf+1:end,:); % actuator torques
end

Q = [q;dq]; % Q is q, q_dot for all timesteps
toc %end timing setup time
%%
disp_box('Building Cost Function');
tic; % Start Timing Cost Function Build

%Set weights
option.alpha = 1; % Weight for joint angle error from reference
option.alpha_f = 100; % Weight for terminal joint angle error
option.alpha_tau = .05; % Weight for control effort

qerr_term = q(4:end,N) - option.qf(4:end); % terminal joint angle error
J = option.alpha_f*(qerr_term')*qerr_term;

%Build Running Cost
deleteStr = '';
for k=1:N-1
    msg = ['Building Cost Step ' num2str(k) ' of ' num2str(N-1)];
    fprintf([deleteStr, msg]);
    deleteStr = repmat(sprintf('\b'), 1, length(msg));
    qerr = q(4:end,k) - [option.q_ref]; %joint error at timestep k
    tau_k = tau_joint(:,k); %control effort at timestep k
    J = J + option.alpha*(qerr')*qerr + option.alpha_tau * (tau_k')*tau_k; % Add terms for joint error and control effort
end

%Pass Cost Function to CASADI
opti.minimize(J);
fprintf('\nComplete!\n')
toc;

%% Constraints
disp_box('Building Constraints');
tic;
deleteStr = '';
for k = 1:N-1
    msg = ['Building Constraint ' num2str(k) ' of ' num2str(N-1) ' state: ' num2str(cs(k))];
    fprintf([deleteStr, msg]);
    deleteStr = repmat(sprintf('\b'), 1, length(msg));
    
    %Extract variables at timestep k
    qk = q(:,k); %Position States
    qjk = qk(4:end); %Position State of Revolute Joints
    dqk = dq(:,k); %Velocity or Momentum States
    dqjk = dqk(4:end); %Velocity or Momentum States for Revolute Joints 
    Qk = [qk;dqk];

    %Compute Position of Joints and Foot
    pf1k = pf1(qk); % Position of front foot contact (toe)
    pf2k = pf2(qk); % Position of rear foot contact (heel)
    pj1k = pJ1(qk); % Position of hip joint
    pj2k = pJ2(qk); % Position of knee joint
    pj3k = pJ3(qk); % Position of ankle joint

    %Compute Jacobians
    Jfk = Jf(qk); % Jacobian (combined front and rear contact jacobian)

    %Damping Torque
    tau_d = 1*Kd_j*Qk(nx+1:end);

%% Dynamics Constraints
    %% Euler
    if INTEGRATION_METHOD == "Euler"
        Mk = M(qk);    % mass matrix
        Ck = C(Qk); % bias torques
        if(cs(k+1) == 0)% if in flight, constraints are just ddq, M
            Ak = Mk;
            xk = ddq(:,k);
        else %if in ground contact constraints now include rear reaction force and jacobian
            Ak = [Mk, Jfk.'];
            xk = [ddq(:,k);f_r(:,k)];
        end
        % bek is always torque (bias + damping + motor)
        % the body coordinates aren't actuated
        bk =-Ck - tau_d + [0;0;0;tau_joint(:,k)];

        %Euler Dynamics Constraint
        opti.subject_to(Ak * xk == bk);
        % euler integration of dq,q
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
        %VI Requires states at timestep k+1
        qkp1 = q(:,k+1);
        dqkp1 = dq(:,k+1);

        %Jacobian at timestep k+1
        Jfkp1 = Jf(qkp1); %Jacobian at k+1        

        %Define Midpoint Variables
        q_mp = (qk+qkp1)/2;
        dq_mp = (qkp1-qk)/dt;
        Q_mp = [q_mp;dq_mp];

        tau_d = 1*Kd_j*dq_mp;

        % Define left and right discrete forces
        f_d0 = (-tau_d+[0;0;0;tau_joint(:,k)] + (Jfk.')*(-1*f_r(:,k)));
        f_d1 = (-tau_d+[0;0;0;tau_joint(:,k)] + (Jfkp1.')*(-1*f_r(:,k+1)));
        
        % Define Implicit VI Update Rules
        p0VI = p0(Q_mp,dt) - 0.5*dt*f_d0;
        p1VI = p1(Q_mp,dt) + 0.5*dt*f_d1;
        opti.subject_to(dqk==p0VI);
        opti.subject_to(dqkp1==p1VI);
    end

%% Contact Constraints
    if cs(k+1) == 0 % If in flight
        opti.subject_to(f_r(:,k) == [0;0;0;0]); % ground reaction forces must be zero
    elseif cs(k+1) == 1 % If single contact
        opti.subject_to(f_r(3:4,k) == [0;0]);
        opti.subject_to(f_r(2,k) <= -10);
        opti.subject_to(f_r(2,k) >= -300);
    else % If on the ground
        opti.subject_to(f_r(2,k) <= -10); % Choose some minimum value of normal force to prevent slip
        opti.subject_to(f_r(2,k) >= -300);
        %repeat constraints for rear force
        opti.subject_to(f_r(4,k) <= -10);
        opti.subject_to(f_r(4,k) >= -300);
    end

    if cs(k) == 1
        opti.subject_to((pf1k - pf1i) == [0;0]); % front foot contact must be on the ground
        opti.subject_to((pf2k(2) - pf2i(2)) >= 0); %rear foot contact must be above the ground
    elseif cs(k) == 2
        opti.subject_to((pf1k - pf1i) == [0;0]); % front foot contact must be on the ground
        opti.subject_to((pf2k - pf2i) == [0;0]); % rear foot contact must be on the ground
    end

    % friction cone
    opti.subject_to(f_r(1,k) <= -option.mu*f_r(2,k));
    opti.subject_to(f_r(1,k) >= option.mu*f_r(2,k));
    opti.subject_to(f_r(3,k) <= -option.mu*f_r(4,k));
    opti.subject_to(f_r(3,k) >= option.mu*f_r(4,k));
%% Motion and Kinematic Constraints
    if cs(k) ~= 0 % If in contact with the ground
        opti.subject_to(pj1k(2) - pf1k(2) >= 0.07); % Position of hip joint is above the ground
        opti.subject_to(pj2k(2) - pf1k(2) >= 0.05); % Position of knee joint is above the ground
        opti.subject_to(qk(3) >= (-pi/6));
        opti.subject_to(qk(3) <= ( pi/6));
    end

    %Joint Angle Limits
    qj_min = [option.q_joint_limit_hip(1); option.q_joint_limit_knee(1); option.q_joint_limit_ankle(1)];
    qj_max = [option.q_joint_limit_hip(2); option.q_joint_limit_knee(2); option.q_joint_limit_ankle(2)];
    opti.subject_to(qjk <= qj_max);
    opti.subject_to(qjk >= qj_min);

    %Joint Velocity Limits
    if INTEGRATION_METHOD == "VI"
        opti.subject_to(dq_mp(4:end) <= ones([nx-3 1])*option.joint_vel_limit);
        opti.subject_to(dq_mp(4:end) >= -1*ones([nx-3 1])*option.joint_vel_limit);
    else
        opti.subject_to(dqjk <= ones([nx-3 1])*option.joint_vel_limit);
        opti.subject_to(dqjk >= -1*ones([nx-3 1])*option.joint_vel_limit);
    end
%% Control Effort Limits
    if(cs(k) ~= 0)
        %Contact Phase
        opti.subject_to(tau_joint(:,k) <= 2*option.torque_saturation_stance);
        opti.subject_to(tau_joint(:,k) >= -2*option.torque_saturation_stance);
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
opti.subject_to(q(:,N) == option.qf);
opti.subject_to(f_r(:,N) == zeros([ncf 1]));
opti.subject_to(tau_joint(:,N) == zeros([nu 1]));

%Momentum Constraints if VI, else Velocity Constraints
if INTEGRATION_METHOD == "VI"
    opti.subject_to(dq(4:end,N) == VI_JointTermCon([q(:,N);dq(1:3,N)]))
else
    opti.subject_to(dq(4:end,N) == zeros(nx-3,1))
end
fprintf('\nComplete!\n')
toc;

%% Set Initial Guess
posGuess = repmat(option.qi,1,N);
opti.set_initial(q,posGuess);
opti.set_initial(tau_joint,repmat(2*ones(nu,1),1,N));

%% Solve
disp_box('Starting IPOPT');
p_opts = struct('expand',true);
%s_opts = struct('max_iter',2000, 'linear_solver', 'ma27');
s_opts = struct('max_iter',2000);
opti.solver('ipopt',p_opts,s_opts);
tic;
sol = opti.solve();
toc;

%% Plot
contactForces = opti.debug.value(f_r);
pos = opti.debug.value(q);
vel = opti.debug.value(dq);
control = opti.debug.value(tau_joint);
Q_3Link = [pos;vel];

figure(1)
clf()
hold on
for k = 1:nx
    plot(pos(k,:))
end
hold off
legend(["x", "y", "th_b", "hip","knee", "ankle"])
%%
figure(2)
clf()
hold on
for k = 1:nu
    plot(control(k,:))
end
hold off
%%
for k = 1:N
    foot1Pos(:,k) = pf1(pos(:,k));
    foot2Pos(:,k) = pf2(pos(:,k));
    joint1Pos(:,k) = pJ1(pos(:,k));
    joint2Pos(:,k) = pJ2(pos(:,k));
    joint3Pos(:,k) = pJ3(pos(:,k));
end

figure(3)
clf()
hold on
plot(pos(1,:), pos(2,:))
plot(foot1Pos(1,:), foot1Pos(2,:))
plot(foot2Pos(1,:), foot2Pos(2,:))
plot(joint1Pos(1,:), joint1Pos(2,:))
plot(joint2Pos(1,:), joint2Pos(2,:))
plot(joint3Pos(1,:), joint3Pos(2,:))
hold off
axis equal
%%
figure();
clf();
hold on
plot(contactForces(1,:));
plot(contactForces(2,:));
plot(contactForces(3,:));
plot(contactForces(4,:));

%%
pos = opti.debug.value(q);
%animate_3Link_With_Foot(pos);
%% Simulink Visual Setup
W1 = 0.114;
H1 = 0.194;

hip1_offset_y = 0.047;
hip2_offset_y = 0.0185;
thigh_offset_y = 0.004;
calf_offset_y = 0.003;
toe_offset_y = 0.011;

HipJointYOffset = -hip1_offset_y-hip2_offset_y-thigh_offset_y-0.014;

% Generate Foot Vertices for Simulink Display
wF = 0.05; % Foot Width

v1 = [-wF/2 0];
v2 = [wF/2 0];
v3 = (robotParams.pfc1).' + [wF/2 0];
v4 = (robotParams.pfc1).' - [wF/2 0];
v6 = (robotParams.pfc2).' + [wF/2 0];
v7 = (robotParams.pfc2).' - [wF/2 0];

mf1 = robotParams.pfc1(2)/robotParams.pfc1(1);
mf2 = robotParams.pfc2(2)/robotParams.pfc2(1);

xFInt = (mf2*v6(1) - mf1*v4(1))/(mf2-mf1);
yFInt = v6(2) + mf2*(xFInt-v6(1));

v5 = [xFInt yFInt];
 
footVertices = [v7; v6; v5; v4; v3; v2; v1];
clear v1 v2 v3 v4 v5 v6 v7

%% Simulink Simulation

pos = opti.debug.value(q);
Q_3Link = [pos;vel];
%%
%Check which version of Simulink is installed
verStruct = ver;

idx_SL = find(ismember({verStruct.Name}, 'Simulink'));

if isempty(idx_SL)
    error('Simulink not detected on machine.\nPlease install Simulink (2023b or newer) to run simulation of TO result', 0)
else
    SL_Version = str2double(verStruct(idx_SL).Version);
    SL_Release = verStruct(idx_SL).Release;
    if SL_Version >= 23.2
        out = sim(fullfile('..','SimulinkModels','HectorSimulink','Hector_Simulation.slx'), "StopTime", num2str((N-1)*dt));
    else
        error('Installed Simulink version %s is not currently supported.\nPlease install Simulink 2023b or newer, or contact author for support', SL_Release);
    end
end

%% Simulink Plotting

tTO = (0:max(size(pos))-1)*dt;

qSim = out.qout(:, [3:5, 8:10]);
xSim = [out.compTrunkOut,out.xout(:,2)];

XSim = [xSim, qSim].';

XSim(1,:) = XSim(1,:) - (XSim(1,1) - pos(1,1));
XSim(2,:) = XSim(2,:) - (XSim(2,1) - pos(2,1));

flipIdx = find(abs(out.xout(:,1))>pi/2,1);
flipBackIdx = find(abs(out.xout(flipIdx+1:end))<pi/2,1) + flipIdx;
flipIdx2 = find(abs(out.xout(flipBackIdx+1:end,1))>pi/2,1)+flipBackIdx;
flipBackIdx2 = find(abs(out.xout(flipIdx2+1:end))<pi/2,1) + flipIdx2;

XSim(3,flipIdx:flipBackIdx) = 2*XSim(3,flipIdx)-XSim(3,flipIdx:flipBackIdx);
XSim(3,flipIdx2:flipBackIdx2) = 2*XSim(3,flipIdx2)-XSim(3,flipIdx2:flipBackIdx2);
XSim(3,:) = unwrap(XSim(3,:));

%
if INTEGRATION_METHOD == "Euler"
    titles = ["Body X - Euler", "Body Y - Euler", "Body Theta - Euler", "R Hip Angle - Euler", "R Knee Angle - Euler", "R Ankle Angle - Euler", "L Hip Angle - Euler", "L Knee Angle - Euler", "L Ankle Angle - Euler"];
end

if INTEGRATION_METHOD == "RK4"
    titles = ["Body X - RK4", "Body Y - RK4", "Body Theta - RK4", "R Hip Angle - RK4", "R Knee Angle - RK4", "R Ankle Angle - RK4", "L Hip Angle - RK4", "L Knee Angle - RK4", "L Ankle Angle - RK4"];
end

if INTEGRATION_METHOD == "VI"
    titles = ["Body X - VI", "Body Y - VI", "Body Theta - VI", "R Hip Angle - VI", "R Knee Angle - VI", "R Ankle Angle - VI", "L Hip Angle - VI", "L Knee Angle - VI", "L Ankle Angle - VI"];
end
yLabels = ["X (m)", "Y (m)", "Theta (rad)", "q1 (rad)", "q2 (rad)", "q3 (rad)", "q4 (rad)", "q5 (rad)", "q6 (rad)"];

for ii = 1:3
    %min(size(XSim))
    figure()
    clf()
    hold on
    plot(out.tout, XSim(ii,:), '-b', 'LineWidth', 1.5);
    if ii > 6
        plot(tTO, pos(ii-3,:), '--r', 'LineWidth', 1.5);
    else
        plot(tTO, pos(ii,:), '--r', 'LineWidth', 1.5);
    end
    xlabel('Time (s)');
    ylabel(yLabels(ii));
    title(titles(ii));
    legend("Simulated", "TO Reference", 'Location','northwest')
end
tSim = out.tout;

if INTEGRATION_METHOD == "Euler"
    save("EulerForwardResults.mat", "pos","tTO", "XSim", "tSim")
end

if INTEGRATION_METHOD == "VI"
    tSimVI = tSim;
    XSimVI = XSim;
    posVI = pos;
    tTOVI = tTO;
    save("VIForwardResults.mat", "posVI","tTOVI", "XSimVI", "tSimVI")
end

% %%
% 
% figure()
% hold on
% 
% plot(XSim(1,:), XSim(2,:), '-b', 'LineWidth', 1.5);
% plot(pos(1,:), pos(2,:), '--r', 'LineWidth', 1.5);
% hold off
% axis equal
% 
% if INTEGRATION_METHOD == "VI"
%     title("Trajectory of Body CG ")

