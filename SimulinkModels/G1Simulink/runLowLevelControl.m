function tau = runLowLevelControl(x_fb, f_ff, f_swing, q_leg, t, mpc, contact)
q = q_leg; % Joint states
eul = x_fb(1:3);

% R = Rz(eul(3))*Ry(eul(2))*Rx(eul(1)); % rotation matrix
R = eul2rotm(flip(eul'));
RR=reshape(R,[9,1]);

global Contact_Jacobian
Jc=Contact_Jacobian(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),q(11),q(12),...
    RR(1),RR(2),RR(3),RR(4),RR(5),RR(6),RR(7),RR(8),RR(9));

tau = [ [Jc(1:3,:)',Jc(4:6,:)']*([-f_ff(1:3); -f_ff(7:9)]+f_swing(1:6));
        [Jc(7:9,:)',Jc(10:12,:)']*([-f_ff(4:6); -f_ff(10:12)]+f_swing(7:12)) ];

end