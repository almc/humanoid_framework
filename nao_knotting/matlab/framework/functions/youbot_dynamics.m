function [pos_rot, alpha_arm1, alpha_arm2] = youbot_dynamics(base, arm1, arm2, dt)
% YOUBOT_DYNAMICS calculates the next state of the robot
%   alpha: current joint angles of the arms.
%   omega: current wheel velocities.
%   dt: time step discretization

W = base.w;
L = base.l;
R = base.r;

lin_to_ang_vel = 1/R*[1,  1, -(L+W)/2;
                      1, -1, -(L+W)/2;
                      1,  1,  (L+W)/2;
                      1, -1,  (L+W)/2];
                  
ang_to_lin_vel = pinv(lin_to_ang_vel);
lin_vel_rot    = ang_to_lin_vel * base.omega;

% lin_vel_rot = lin_to_ang_vel \ omega;

% arm1.alpha_v(arm1.alpha_v >  0.2) =  0.2;
% arm1.alpha_v(arm1.alpha_v < -0.2) = -0.2;

% arm2.alpha_v(arm2.alpha_v >  0.2) =  0.2;
% arm2.alpha_v(arm2.alpha_v < -0.2) = -0.2;

% lin_vel_rot
% lin_vel_rot_lim = [0.1, 0.1, 0.2]';
% lin_vel_rot(lin_vel_rot >    lin_vel_rot_lim) =    lin_vel_rot_lim(lin_vel_rot >    lin_vel_rot_lim);
% lin_vel_rot(lin_vel_rot < -1*lin_vel_rot_lim) = -1*lin_vel_rot_lim(lin_vel_rot < -1*lin_vel_rot_lim);

pos_rot = base.pos_rot + dt*lin_vel_rot';
alpha_arm1  = wrapToPi(arm1.alpha   + dt*arm1.alpha_v);
alpha_arm2  = wrapToPi(arm2.alpha   + dt*arm2.alpha_v);

%% limiting the angles of the joints
% alpha_arm1(alpha_arm1 > arm1.alpha_l2) = arm1.alpha_l2(alpha_arm1 > arm1.alpha_l2);
% alpha_arm1(alpha_arm1 < arm1.alpha_l1) = arm1.alpha_l1(alpha_arm1 < arm1.alpha_l1);

% alpha_arm2(alpha_arm2 > arm2.alpha_l2) = arm2.alpha_l2(alpha_arm2 > arm2.alpha_l2);
% alpha_arm2(alpha_arm2 < arm2.alpha_l1) = arm2.alpha_l1(alpha_arm2 < arm2.alpha_l1);

end
