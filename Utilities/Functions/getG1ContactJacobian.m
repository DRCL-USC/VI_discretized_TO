function [Contact_Jacobian, Rotm_foot]=getG1ContactJacobian

% Input
q=sym('q',[12,1]);
q_R=q(1:6,1);
q_L=q(7:12,1);
R=sym('r',[9,1]); % input R as vector
R=[reshape(R,[3,3]),zeros(3,1);zeros(1,3),1];

% Right leg configuration
link=repmat({zeros(3,1)},6,1);
link{1} = -[-0.0039635, 0, 0.054]' + [0, 0.064452, -0.1027]'; % com to pitch1
link{2} = [0, 0.052, -0.030465]'; % pitch 1 to roll
link{3} = [0.025001, 0, -0.12412]'; % roll to yaw
link{4} = [-0.078273, 0.0021489, -0.17734]'; % yaw to pitch 2
link{5} = [0, 0, -0.3000]'; % pitch 2 to pitch 3
link{6} = [0, 0, -0.017558]'; % pitch 3 to roll

foot=[0;0;-0.04;1];

% Translation matrices
T_R=repmat({eye(4)},6,1);
T_L=repmat({eye(4)},6,1);
for i=1:6
    T_R{i}(1:3,4)=link{i};
    T_L{i}(1:3,4)=[1;-1;1].*link{i};
end

% Rotation matrices
%R_R=repmat({eye(4)},5,1);
%R_L=repmat({eye(4)},5,1);
R_R{1}=[cos(q_R(1)),0,sin(q_R(1)),0;0,1,0,0;-sin(q_R(1)),0,cos(q_R(1)),0;0,0,0,1];
R_R{2}=[1,0,0,0;0,cos(q_R(2)),-sin(q_R(2)),0;0,sin(q_R(2)),cos(q_R(2)),0;0,0,0,1];
R_R{3}=[cos(q_R(3)),-sin(q_R(3)),0,0;sin(q_R(3)),cos(q_R(3)),0,0;0,0,1,0;0,0,0,1];
R_R{4}=[cos(q_R(4)),0,sin(q_R(4)),0;0,1,0,0;-sin(q_R(4)),0,cos(q_R(4)),0;0,0,0,1];
R_R{5}=[cos(q_R(5)),0,sin(q_R(5)),0;0,1,0,0;-sin(q_R(5)),0,cos(q_R(5)),0;0,0,0,1];
R_R{6}=[1,0,0,0;0,cos(q_R(6)),-sin(q_R(6)),0;0,sin(q_R(6)),cos(q_R(6)),0;0,0,0,1];

R_L{1}=[cos(q_L(1)),0,sin(q_L(1)),0;0,1,0,0;-sin(q_L(1)),0,cos(q_L(1)),0;0,0,0,1];
R_L{2}=[1,0,0,0;0,cos(q_L(2)),-sin(q_L(2)),0;0,sin(q_L(2)),cos(q_L(2)),0;0,0,0,1];
R_L{3}=[cos(q_L(3)),-sin(q_L(3)),0,0;sin(q_L(3)),cos(q_L(3)),0,0;0,0,1,0;0,0,0,1];
R_L{4}=[cos(q_L(4)),0,sin(q_L(4)),0;0,1,0,0;-sin(q_L(4)),0,cos(q_L(4)),0;0,0,0,1];
R_L{5}=[cos(q_L(5)),0,sin(q_L(5)),0;0,1,0,0;-sin(q_L(5)),0,cos(q_L(5)),0;0,0,0,1];
R_L{6}=[1,0,0,0;0,cos(q_L(6)),-sin(q_L(6)),0;0,sin(q_L(6)),cos(q_L(6)),0;0,0,0,1];

r_R=foot;
r_L=foot;
o_R{1}=R(1:3,1:3);
o_L{1}=R(1:3,1:3);
for i=6:-1:1
    r_R=T_R{i}*R_R{i}*r_R;
    r_L=T_L{i}*R_L{i}*r_L;
    o_R{8-i}=o_R{7-i}*R_R{7-i}(1:3,1:3);
    o_L{8-i}=o_L{7-i}*R_L{7-i}(1:3,1:3);
end
r_R=R*r_R;
r_L=R*r_L;

% Jc=[dr;do]/dq
Jr_R=jacobian(r_R(1:3,1),q_R);
Jr_L=jacobian(r_L(1:3,1),q_L);

Jo_R=[o_R{1}*[0;1;0],o_R{2}*[1;0;0],o_R{3}*[0;0;1],o_R{4}*[0;1;0],o_R{5}*[0;1;0],o_R{6}*[1;0;0]];
Jo_L=[o_L{1}*[0;1;0],o_L{2}*[1;0;0],o_L{3}*[0;0;1],o_L{4}*[0;1;0],o_L{5}*[0;1;0],o_L{6}*[1;0;0]];
Jc=[Jr_R; Jo_R; Jr_L; Jo_L]; 
Contact_Jacobian=matlabFunction(Jc);

Rotm_foot=matlabFunction([o_R{7};o_L{7}]); % additional output, foot orientation, size=[6,3]


end