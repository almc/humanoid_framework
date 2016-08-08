function [] = draw_youbot(base, arm1, arm2)
% DRAW_YOUBOT draws the youbot arms, base and wheels.

%% setting view position and axis limits
%subplot(1,2,1);
% hold on, grid on
% view([4 2 2]);
% axis([-4 +4, -4 +4, +0 +4]);

%% drawing arms and base
draw_base(base.pos_rot, base.w, base.l);
draw_arm (arm1.ef, arm1.j1, arm1.j2, arm1.j3, arm1.j4, arm1.j5, 1);
draw_arm (arm2.ef, arm2.j1, arm2.j2, arm2.j3, arm2.j4, arm2.j5, 2);

%% setting figure caption
% a1 = alpha(1)*180/pi;     a2 = alpha(2)*180/pi;     a3 = alpha(3)*180/pi;
% c1 = num2str(a1,'%6.2f'); c2 = num2str(a2,'%6.2f'); c3 = num2str(a3,'%6.2f');
% title_string = ['\alpha_1: ', c1, '  \alpha_2: ', c2, '  \alpha_3: ', c3];
% title(title_string);
% title ('Youbot Knotting Simulation');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

end

