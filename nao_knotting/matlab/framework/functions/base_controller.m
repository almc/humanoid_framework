function [omega] = base_controller(pos_rot, base_ref, Kp_base, base)

x = pos_rot(1); y = pos_rot(2); rot = pos_rot(3);
err_x   = base_ref(1) - x;
err_y   = base_ref(2) - y;
err_rot = wrapToPi(base_ref(3) - rot);

%% reading controller constants
k_x   = Kp_base.x;
k_y   = Kp_base.y;
k_rot = Kp_base.rot;

%% performance intensive method
% v_x   = +k_x*err_x*cos(rot)*sign(cos(rot)) + k_y*err_y*sin(rot)
% v_y   = -k_x*err_x*sin(rot) + k_y*err_y*cos(rot)*sign(sin(rot))
% v_rot = +k_rot*err_rot;

world_to_youbot = [ cos(rot), -sin(rot), 0 ;
                    sin(rot),  cos(rot), 0 ;
                    0       ,  0       , 1];

err_xy_youbot = world_to_youbot \ [err_x, err_y, 0]';
v_x_youbot    = k_x   * err_xy_youbot(1);
v_y_youbot    = k_y   * err_xy_youbot(2);
v_rot         = k_rot * err_rot;
v_xy_world    = world_to_youbot * [v_x_youbot, v_y_youbot, 0]';
v_x = v_xy_world(1);
v_y = v_xy_world(2);

% err_ag_youbot = atan2(err_xy_youbot(2), err_xy_youbot(1))
% v_rot = k_rot * err_ag_youbot;

%% trigonometric method
% beta  = atan2(err_y, err_x);
% alfa  = rot;
% gama  = beta - alfa;
% error = sqrt(power(err_x,2) + power(err_y,2));
% v_x   = k_x*error*cos(gama);
% v_y   = k_y*error*sin(gama);


% text = ['v_x', num2str(v_x), 'v_y', num2str(v_y), 'v_rot', num2str(v_rot)];
% text = ['err_x', num2str(err_x), 'err_y', num2str(err_y), 'err_rot', num2str(err_rot)];
% disp(text)

%norm1 = sqrt(e_x**2+e_y**2)
%norm2 = abs(e_psi)

W = base.w;
L = base.l;
R = base.r;

lin_to_ang_vel = 1/R*[1,  1, -(L+W)/2;
                      1, -1, -(L+W)/2;
                      1,  1,  (L+W)/2;
                      1, -1,  (L+W)/2];

omega = lin_to_ang_vel * [v_x, v_y, v_rot]';

end

