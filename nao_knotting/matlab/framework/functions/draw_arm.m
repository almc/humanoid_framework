function [] = draw_arm(ef, j1, j2, j3, j4, j5, arm_id)
% DRAW_ARM draws a given arm of the youbot.

%% discrete line segments to draw the arm
grid_n = 20; a = 2; b = 19;
x1=linspace(j1(1),j2(1),grid_n); x1=x1(a:b);
x2=linspace(j2(1),j3(1),grid_n); x2=x2(a:b);
x3=linspace(j3(1),j4(1),grid_n); x3=x3(a:b);
x4=linspace(j4(1),j5(1),grid_n); x4=x4(a:b);
x5=linspace(j5(1),ef(1),grid_n); x5=x5(a:b);

y1=linspace(j1(2),j2(2),grid_n); y1=y1(a:b);
y2=linspace(j2(2),j3(2),grid_n); y2=y2(a:b);
y3=linspace(j3(2),j4(2),grid_n); y3=y3(a:b);
y4=linspace(j4(2),j5(2),grid_n); y4=y4(a:b);
y5=linspace(j5(2),ef(2),grid_n); y5=y5(a:b);

z1=linspace(j1(3),j2(3),grid_n); z1=z1(a:b);
z2=linspace(j2(3),j3(3),grid_n); z2=z2(a:b);
z3=linspace(j3(3),j4(3),grid_n); z3=z3(a:b);
z4=linspace(j4(3),j5(3),grid_n); z4=z4(a:b);
z5=linspace(j5(3),ef(3),grid_n); z5=z5(a:b);

%% drawing joints and end effector
plot3(j1(1),j1(2),j1(3),'ko','MarkerFaceColor','w', 'MarkerSize', 4);
plot3(j2(1),j2(2),j2(3),'ko','MarkerFaceColor','w', 'MarkerSize', 4);
plot3(j3(1),j3(2),j3(3),'ko','MarkerFaceColor','w', 'MarkerSize', 4);
plot3(j4(1),j4(2),j4(3),'ko','MarkerFaceColor','w', 'MarkerSize', 4);
plot3(j5(1),j5(2),j5(3),'ko','MarkerFaceColor','w', 'MarkerSize', 4);
if     arm_id == 1
    plot3(ef(1),ef(2),ef(3),'ko','MarkerFaceColor','r', 'MarkerSize', 4);
elseif arm_id == 2
    plot3(ef(1),ef(2),ef(3),'ko','MarkerFaceColor','b', 'MarkerSize', 4);
end
% plot3(ef(1),ef(2),    0,'ko','MarkerFaceColor','k'); % shadow of ef

%% drawing the arm of the robot
plot3(x1,y1,z1,'y','LineWidth',4);
plot3(x2,y2,z2,'y','LineWidth',4);
plot3(x3,y3,z3,'y','LineWidth',4);
plot3(x4,y4,z4,'y','LineWidth',4);
plot3(x5,y5,z5,'y','LineWidth',4);

plot3(x1,y1,z1,'k','LineWidth',1);
plot3(x2,y2,z2,'k','LineWidth',1);
plot3(x3,y3,z3,'k','LineWidth',1);
plot3(x4,y4,z4,'k','LineWidth',1);
plot3(x5,y5,z5,'k','LineWidth',1);

end

