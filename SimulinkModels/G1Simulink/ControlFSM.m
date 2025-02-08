function tau = ControlFSM(uin)
global f_ff
x_fb = uin(1:12);
q_leg = uin(13:24);
qd_leg = uin(25:36);
foot_l = uin(37:48);
foot_r = uin(49:60);
t = uin(end)
counter = floor(t*1000);

%initialization
if t == 0
    f_ff = zeros(12,1);
end

% control parameters
mpc.x_cmd = [0; 0+0*sin(pi*t);0;
             0;0;0.8+0*sin(pi*t/2); 
             0;0;0; 
             0.5;0;0];
mpc.dt = 0.05; % native mpc frequency
mpc.h = 10;
mpc.g = 9.81;
mpc.mu = 0.7;
mpc.hyperSampling = 10; 
mpc.Q = [500 500 300, 200 600 400, 1 1 1, 1 1 1, 1];
mpc.R = ones(1,12)*1e-5;
mpc.f_max = [1000; 1000; 1000];
mpc.f_min = [0; 0; 0];
mpc.m_max = [40; 40; 40];
mpc.m_min = -mpc.m_max;
mpc.kp_l = diag([1,1,1])*1000;
mpc.kd_l = diag([1,1,1])*3;
mpc.kp_r = diag([0,1,0])*100;
mpc.kd_r = diag([0,1,0])*2;
mpc.swingHeight = 0.2;
mpc.kv = 0.03;

%% gait generator
if t <= 0
    gaitNumber = 0;
else
    gaitNumber = 1;
end
contact = gait(gaitNumber, t, mpc)

%% MPC
if rem(counter, 1000*mpc.dt/mpc.hyperSampling) == 0 
    f_ff = runMPCWrapper(x_fb, q_leg, foot_l, foot_r, t, mpc, contact)
end

%% Low level control
f_swing = runSwingControl(x_fb, q_leg, foot_l, foot_r, t, mpc, contact)

tau = runLowLevelControl(x_fb, f_ff, f_swing, q_leg, t, mpc, contact)

% tau = zeros(12,1);
end


%%%%%%%%%%%%%%%  function %%%%%%%%%%%%%%%%%%%%%%%%
function contact = gait(n,t, mpc)
    if n == 0
        contact = ones(mpc.h, 2);
    elseif n == 1
        contact = getContactSequence(t, mpc); 
    end
end

function contact = getContactSequence(t, mpc)
    contact = [1,1,1,1,1, 0,0,0,0,0, 1,1,1,1,1, 0,0,0,0,0, 1,1,1,1,1, 0,0,0,0,0, 1,1,1,1,1, 0,0,0,0,0;
               0,0,0,0,0, 1,1,1,1,1, 0,0,0,0,0, 1,1,1,1,1, 0,0,0,0,0, 1,1,1,1,1, 0,0,0,0,0, 1,1,1,1,1]';
    % contact = [1,1,1,1,1, 0,0,1,1,1, 1,1,1,1,1, 0,0,1,1,1;
    %            0,0,1,1,1, 1,1,1,1,1, 0,0,1,1,1, 1,1,1,1,1]';
    % contact = [1,1,1,1,1, 1,1,1,1,1, 0,0,0,0,0, 1,1,1,1,1, 1,1,1,1,1, 0,0,0,0,0;
    %            0,0,0,0,0, 1,1,1,1,1, 1,1,1,1,1, 0,0,0,0,0, 1,1,1,1,1, 1,1,1,1,1]';
    phase = floor(t/mpc.dt);
    k = rem(phase,mpc.h)+1
    contact = contact(k:k+mpc.h-1, :);
end