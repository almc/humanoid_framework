function [j1, j2, j3, j4, j5, ef] = fk_youbot(alpha, pos_rot, J1, J2, J3, J4, J5, EF)
% FK_YOUBOT forward kinematics of the youbot
%   alpha: input array of angles in radians

a1 = alpha(1);   a2 = alpha(2);   a3 = alpha(3);   a4 = alpha(4);   a5 = alpha(5);
px = pos_rot(1); py = pos_rot(2); rt = pos_rot(3);

j1 = eval(J1);
j2 = eval(J2);
j3 = eval(J3);
j4 = eval(J4);
j5 = eval(J5);
ef = eval(EF);

end

