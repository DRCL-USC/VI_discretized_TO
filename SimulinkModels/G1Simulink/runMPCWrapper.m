function f_ff = runMPCWrapper(x_fb, q_leg, foot_l, foot_r, t, mpc, contact)
global Rotm_foot
import casadi.*
h = mpc.h;
dt = mpc.dt;
g = mpc.g;

% enforce "2*pi=0" relation for turning
eul = x_fb(1:3);
eul_des = mpc.x_cmd(1:3);
R = eul2rotm(flip(eul'));
yaw_correction=0;
RR=reshape(R,[9,1]);
R_foot=Rotm_foot(q_leg(1),q_leg(2),q_leg(3),q_leg(4),q_leg(5),q_leg(6),q_leg(7),q_leg(8),q_leg(9),q_leg(10),q_leg(11),q_leg(12),RR(1),RR(2),RR(3),RR(4),RR(5),RR(6),RR(7),RR(8),RR(9));
R_foot_R=R_foot(1:3,:);
R_foot_L=R_foot(4:6,:);

R_foot_R=R;
R_foot_L=R;

while yaw_correction==0
    if eul_des(3,1)-eul(3,1)>pi
        eul(3,1)=eul(3,1)+2*pi;
    elseif eul_des(3,1)-eul(3,1)<-pi
        eul(3,1)=eul(3,1)-2*pi;
    else
        yaw_correction=1;
    end
end

% rearrange MPC states
mpc.x_cmd = [mpc.x_cmd;g];
x_fb = [x_fb;g];

%% Assigning desired trajectory for CoM and Foot locations
foot_ref = getReferenceFootTrajectory(...
    x_fb, t, mpc, foot_l, foot_r, contact)
x_ref = getReferenceTrajectory(x_fb, mpc)

%% Robot simplified dynamics physical properties 
mu = mpc.mu; %friction coefficient
m = 35; % mass (inclulded body, hips, and thighs)
Ib = diag([0.9413, 0.9200, 0.2691])*1; % SRBD MoI (included body, hips, and thighs)

%% State-space A & B matrices  (dynamics are done in world frame)
B=repmat({zeros(13,12)},h,1);
A_hat=repmat({zeros(13,13)},h,1);

% continuous A & B:
for i = 1 : h
    Ri = eul2rotm(flip(x_ref(1:3,i)'));
    I = Ri*Ib*Ri'; % MoI in world frame
    S_R = [cos(x_ref(3,i))*cos(x_ref(2,i)),-sin(x_ref(3,i)),0;
        sin(x_ref(3,i))*cos(x_ref(2,i)),cos(x_ref(3,i)),0;
        -sin(x_ref(2,i)),0,1]\eye(3);
    Ac=[zeros(3,3), zeros(3,3), S_R, zeros(3,3), zeros(3,1);
        zeros(3,3), zeros(3,3), zeros(3,3), eye(3), zeros(3,1);
        zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,1);
        zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3),[0;0;-1];
        zeros(1,13)];
    Bc=[zeros(3,3),zeros(3,3),zeros(3,3),zeros(3,3);
        zeros(3,3),zeros(3,3),zeros(3,3),zeros(3,3);
        I\skew2(-x_ref(4:6,i) + foot_ref(1:3,i)), I\skew2(-x_ref(4:6,i) + foot_ref(4:6,i)), I\eye(3), I\eye(3);
        eye(3)/m, eye(3)/m, zeros(3),zeros(3);
        zeros(1,12)];

    % discretization:
    B{i}=Bc*dt;
    A_hat{i}=eye(13)+Ac*dt;
end

%% QP_MPC derivation:
% translating MPC to a condensed QP problem:
% ref: Jerez, Juan L., Eric C. Kerrigan, and George A. Constantinides. 
% "A condensed and sparse QP formulation for predictive control." 
% 2011 50th IEEE Conference on Decision and Control and European Control 
% Conference. IEEE, 2011.

y = reshape(x_ref,[13*h 1]);
Aqp=repmat({zeros(13,13)},h,1);
%Aqp
Aqp{1}=A_hat{1};
for i=2:h
    Aqp{i}=Aqp{i-1}*A_hat{i};
end
Aqp=cell2mat(Aqp);
%Bqp
Bqp=repmat({zeros(13,12)},h,h);
for i=1:h
    Bqp{i,i}=B{i};
    for j=1:h-1
        Bqp{i,j}=A_hat{i}^(i-j)*B{j};
    end
end
for i=1:h-1
    for j=i+1:h
        Bqp{i,j}=zeros(13,12);
    end
end
Bqp=cell2mat(Bqp);

%% MPC Weights: (tune these according to your task)
% state tracking objective:
L1 = mpc.Q; 
% control input minimization:
alpha = mpc.R; %
L10 = repmat(L1,1,h);
L=diag(L10);
alpha10 = repmat(alpha,1,h);
K = diag(alpha10);
% MPC->QP math -> Condensed form
Hd = 2*(Bqp'*L*Bqp+K);
fd = 2*Bqp'*L*(Aqp*x_fb-y);  

%% MPC Constraints:
% please refer to the QPOASES constraint format: lbA <= A*u <= ubA
bigNum = 1e6;
smallNum = -bigNum;

%friction constraint:
A_mu = [1,0,-mu,zeros(1,9);
    0,1,-mu,zeros(1,9); 
    1,0,mu,zeros(1,9); 
    0,1,mu,zeros(1,9);
    zeros(1,3),1,0,-mu,zeros(1,6);
    zeros(1,3),0,1,-mu,zeros(1,6); 
    zeros(1,3),1,0,mu,zeros(1,6); 
    zeros(1,3),0,1,mu,zeros(1,6)];
A_mu_h = kron(eye(h),A_mu);
lba_mu = repmat([smallNum;smallNum;0;0; smallNum;smallNum;0;0],h,1);
uba_mu = repmat([0;0;bigNum;bigNum; 0;0;bigNum;bigNum],h,1);

%force limit constraint:
A_f = eye(12);
A_f_h = kron(eye(h),A_f);
[u_min, u_max] = getSaturations(contact, mpc);
lba_force = u_min; 
uba_force = u_max;

% CCW
lt = 0.16+0.0; lh = 0.06+0.0;% line foot lengths
A_LF1=[-lh*[0,0,1]*R_foot_R',zeros(1,3),[0,1,0]*R_foot_R',zeros(1,3);
    -lt*[0,0,1]*R_foot_R',zeros(1,3),-[0,1,0]*R_foot_R',zeros(1,3);
    zeros(1,3),-lh*[0,0,1]*R_foot_L',zeros(1,3),[0,1,0]*R_foot_L';
    zeros(1,3),-lt*[0,0,1]*R_foot_L',zeros(1,3),-[0,1,0]*R_foot_L'];
A_LF2 = 1*[ [0, lt, -mu*lt]*R_foot_R', zeros(1,3), [0, -mu, -1]*R_foot_R', zeros(1,3);
    zeros(1,3), [0, lt, -mu*lt]*R_foot_L', zeros(1,3), [0, -mu, -1]*R_foot_L';
    [0, -lt, -mu*lt]*R_foot_R', zeros(1,3), [0, -mu, -1]*R_foot_R', zeros(1,3);
    zeros(1,3), [0, -lt, -mu*lt]*R_foot_L', zeros(1,3), [0, -mu, -1]*R_foot_L';
    [0, lh, -mu*lh]*R_foot_R', zeros(1,3), [0, mu, 1]*R_foot_R', zeros(1,3);
    zeros(1,3), [0, lh, -mu*lh]*R_foot_L', zeros(1,3), [0, mu, 1]*R_foot_L';
    [0, -lh, -mu*lh]*R_foot_R', zeros(1,3), [0, mu, -1]*R_foot_R', zeros(1,3);
    zeros(1,3), [0, -lh, -mu*lh]*R_foot_L', zeros(1,3), [0, mu, -1]*R_foot_L'];
uba_LF = repmat(zeros(12,1),h,1); 
lba_LF = repmat(ones(12,1)*smallNum,h,1);
A_LF = [A_LF1;A_LF2]; 
A_LF_h =kron(eye(h), A_LF);

% foot moment Mx=0: 
Moment_selection=[1,0,0];
A_M = [zeros(1,3),zeros(1,3),Moment_selection*R_foot_R',zeros(1,3);
    zeros(1,3),zeros(1,3),zeros(1,3),Moment_selection*R_foot_L'];
A_M_h = kron(eye(h),A_M);
uba_M = zeros(2*h,1);
lba_M = uba_M;

% Constraint Aggregation:
A_size = size([A_mu_h; A_f_h; A_LF_h; A_M_h]);
A = DM(A_size(1),A_size(2)); 
A(:,:) = [A_mu_h; A_f_h; A_LF_h; A_M_h*0]; % row 1-80

lba = [lba_mu; lba_force; lba_LF; lba_M];
uba = [uba_mu; uba_force; uba_LF; uba_M];

%% QPOASES setup:
Hsize = size(Hd);
fsize = size(fd);
H = DM(Hsize);
H = DM(Hd);
g = DM(fsize);
g = DM(fd);

qp = struct;
qp.h = H.sparsity();
qp.a = A.sparsity();
opts = struct('printLevel', 'low');
S = conic('S','qpoases',qp,opts);
% S = conic('S','osqp',qp);
% disp(S)
r = S('h', H, 'g', g, 'a', A, 'lba',lba, 'uba', uba);

% solve!:
tic
x_opt = r.x;
% disp(x_opt(1:12));
disp('QPOASES-MPC Solve Time:');
toc

GRFM = full(x_opt);
GRFM = GRFM(1:12);
% u=-contact_mapping*GRFM(1:12); 
f_ff=[GRFM(1:12)];
disp('QPOASES-MPC Function Total Time:');
toc

end

%%%%%%%%%%%%%%%%%%%%%% fucntions  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function x_ref = getReferenceTrajectory(x_fb, mpc)
    dt = mpc.dt;
    h = mpc.h;
    x_ref = repmat([mpc.x_cmd],1,h);
    x_ref(:,1) = [x_fb];
    for i = 1:6
        for k = 2:h
            if mpc.x_cmd(i+6) ~= 0
                x_ref(i,k) = x_fb(i)+mpc.x_cmd(i+6)*((k-1)*dt);
            else
                x_ref(i,k) = mpc.x_cmd(i);
            end
        end
    end
end

function foot_ref = getReferenceFootTrajectory(...
    x_fb, t, mpc, foot_l, foot_r, contact)
foot_0 = [foot_l(7:9); foot_r(7:9)];
foot_des_x_1 = x_fb(4) + x_fb(10)*1/2*mpc.h/2*mpc.dt ...
    + mpc.kv*(x_fb(10) - mpc.x_cmd(10));
foot_des_x_2 = x_fb(4) + x_fb(10)*1/2*mpc.h/1*mpc.dt ...
    + mpc.kv*(x_fb(10) - mpc.x_cmd(10));
foot_des_y_1 = x_fb(5) + x_fb(11)*1/2*mpc.h/2*mpc.dt ...
    + mpc.kv*(x_fb(11) - mpc.x_cmd(11));
foot_des_y_2 = x_fb(5) + x_fb(11)*1/2*mpc.h/1*mpc.dt ...
    + mpc.kv*(x_fb(11) - mpc.x_cmd(11));
foot_des_z = 0;
foot_1 = [foot_des_x_1; foot_des_y_1; foot_des_z;
    foot_des_x_1; foot_des_y_1; foot_des_z];
foot_2 = [foot_des_x_2; foot_des_y_2; foot_des_z;
    foot_des_x_2; foot_des_y_2; foot_des_z];
kk = floor(t/mpc.dt);
phase = rem(kk,mpc.h);
phase_i = rem(phase,5); %0 1 2 3 4
if sum(contact(1,:)) == 1
    foot_ref = [ repmat(foot_0,1,5-phase_i), ...
        repmat(foot_1,1,5), ...
        repmat(foot_2,1,phase_i)];
else
    foot_ref = repmat(foot_0,1,mpc.h);
end

foot_ref = repmat(foot_0,1,mpc.h);
foot_ref(3,:) = foot_ref(3,:)*0;
foot_ref(6,:) = foot_ref(6,:)*0;

end

function s = skew2(vec)
    s = [0 -vec(3) vec(2);
           vec(3) 0 -vec(1);
           -vec(2) vec(1) 0;];
end

function [u_min, u_max] = getSaturations(contact, mpc)
u_min = []; u_max = [];
    for k = 1:mpc.h
        u_max = [u_max;
            contact(k, 1) * mpc.f_max;
            contact(k, 2) * mpc.f_max;
            contact(k, 1) * mpc.m_max;
            contact(k, 2) * mpc.m_max];
        u_min = [u_min;
            contact(k, 1) * mpc.f_min;
            contact(k, 2) * mpc.f_min;
            contact(k, 1) * mpc.m_min;
            contact(k, 2) * mpc.m_min];
    end
end