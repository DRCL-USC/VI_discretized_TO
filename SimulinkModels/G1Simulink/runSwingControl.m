function f_swing = runSwingControl(x_fb, q_leg, foot_l, foot_r, t, mpc, contact)
global foot_i 
if t == 0
   foot_i = zeros(3,2);
end
phase = floor(t/mpc.dt);
k = rem(phase,mpc.h);
offset = 0.06;
foot_des_x_1 = x_fb(4) + x_fb(10)*1/2*mpc.h/2*mpc.dt ...
    + mpc.kv*(x_fb(10) - mpc.x_cmd(10));
foot_des_x_2 = x_fb(4) + x_fb(10)*1/2*mpc.h/2*mpc.dt ...
    + mpc.kv*(x_fb(10) - mpc.x_cmd(10));
foot_des_y_1 = x_fb(5) + x_fb(11)*1/2*mpc.h/2*mpc.dt ...
    + mpc.kv*(x_fb(11) - mpc.x_cmd(11)) + offset;
foot_des_y_2 = x_fb(5) + x_fb(11)*1/2*mpc.h/2*mpc.dt ...
    + mpc.kv*(x_fb(11) - mpc.x_cmd(11)) - offset;
t = rem(t, mpc.dt*mpc.h/2);
foot_des_z = mpc.swingHeight*sin(pi*t/(mpc.dt*mpc.h/2)) +0.0;
percent = t/(mpc.dt*mpc.h/2);
if percent < 0.01
    foot_i = [foot_l(7:9), foot_r(7:9)];
end

foot_des_l = [foot_des_x_1; foot_des_y_1; foot_des_z];
foot_des_r = [foot_des_x_2; foot_des_y_2; foot_des_z];

foot_des_l = foot_i(:,1) + percent*(foot_des_l - foot_i(:,1));
foot_des_r = foot_i(:,2) + percent*(foot_des_r - foot_i(:,2));

foot_rot_des_l = [0;0;0];
foot_rot_des_r = [0;0;0];

f_swing_l = mpc.kp_l * (foot_des_l - foot_l(7:9)) + ...
            mpc.kd_l * (0 - foot_l(10:12));
f_swing_r = mpc.kp_l * (foot_des_r - foot_r(7:9)) + ...
            mpc.kd_l * (0 - foot_r(10:12));
m_swing_l = mpc.kp_r * (foot_rot_des_l - foot_l(1:3)) + ...
            mpc.kd_r * (0 - foot_l(4:6));
m_swing_r = mpc.kp_r * (foot_rot_des_r - foot_r(1:3)) + ...
            mpc.kd_r * (0 - foot_r(4:6));
contact = -[contact(1,1), contact(1,2)]+1;

f_swing = [f_swing_l*contact(1); m_swing_l*contact(1); 
           f_swing_r*contact(2); m_swing_r*contact(2)];
end