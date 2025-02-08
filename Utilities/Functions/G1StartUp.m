global Contact_Jacobian Rotm_foot
%% robot parameters
% initial states
x_i = [0; 0; 0;   
       0; 0; 0.77;
       0; 0; 0;
       0; 0; 0;
       ];
q_leg_l_i = [-15; 0; 0; 30; -15; 0];
q_leg_r_i = [-15; 0; 0; 30; -15; 0];

% parameters
Kp_ground = 1e5;
Kd_ground = 1e3;
mu = 1.5;
q_damping = 0.2;
kp_arm =100;
kd_arm = 6;

% foot contact approximated by point clouds:
npt = 5; % number of contact points per line
foot_contact_cloud = [ [[linspace(-0.06,0.14,npt)]',ones(npt,1)*0, ones(npt,1)*0]; 
                  [[linspace(-0.06,0.14,npt)]',ones(npt,1)*0.03, ones(npt,1)*0]; 
                  [[linspace(-0.06,0.14,npt)]',ones(npt,1)*-0.03, ones(npt,1)*0]];

% get contact jacobian symbolic
[Contact_Jacobian, Rotm_foot] = getG1ContactJacobian;
