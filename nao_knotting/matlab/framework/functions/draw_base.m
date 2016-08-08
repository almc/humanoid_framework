function [] = draw_base(pos_rot, width, length)
% DRAW_BASE draws the youbot arms, base and wheels.

%% drawing base of the robot
W = width;
L = length;
% R = base.r;
rxy1 = [L, W]'; rxy2 = [-L, W]'; rxy3 = [-L, -W]'; rxy4 = [L, -W]'; rz = 0;
% rotate
T = pos_rot(3);
Rot = [cos(T), -sin(T);
       sin(T),  cos(T)];
rxy1 = Rot * rxy1; rxy2 = Rot * rxy2; rxy3 = Rot * rxy3; rxy4 = Rot * rxy4; 

% translate
rxy1 = rxy1 + pos_rot(1:2)'; rxy2 = rxy2 + pos_rot(1:2)';
rxy3 = rxy3 + pos_rot(1:2)'; rxy4 = rxy4 + pos_rot(1:2)';

patch([rxy1(1) rxy2(1) rxy3(1) rxy4(1) rxy1(1)], ...
      [rxy1(2) rxy2(2) rxy3(2) rxy4(2) rxy1(2)], [rz rz rz rz rz]);

% plot3((rxy1(1) + rxy4(1)) / 2, (rxy1(2) + rxy4(2)) / 2, 0,'ko','MarkerFaceColor','w');
% quiver3(pos_rot(1), pos_rot(2), 0, (rxy1(1) + rxy4(1)) / 2 - pos_rot(1), (rxy1(2) + rxy4(2)) / 2 - pos_rot(2), 0);
% quiver(pos_rot(1), pos_rot(2), (rxy1(1) + rxy4(1)) / 2 - pos_rot(1), (rxy1(2) + rxy4(2)) / 2 - pos_rot(2));
end

