%% visualize loop
plot3(x,y,z,'LineWidth', 4);
plot3([x(end),x(1)], [y(end),y(1)], [z(end),z(1)], 'r-.');
plot3(loop_center(1), loop_center(2), loop_center(3), 'ko','MarkerFaceColor','r');
quiver3(loop_center(1), loop_center(2), loop_center(3), ...
        perp_vector(1), perp_vector(2), perp_vector(3), 'r');
quiver3(loop_center(1) , loop_center(2) , loop_center(3), ...
        parl_vector1(1), parl_vector1(2), parl_vector1(3), 'g');
quiver3(loop_center(1) , loop_center(2) , loop_center(3), ...
        parl_vector2(1), parl_vector2(2), parl_vector2(3), 'b');

%% Draw plane parallel to the loop
% [X_plane,Y_plane] = meshgrid(linspace(-1,1,2));
% surf(X_plane,Y_plane, - (n_1(1)/n_1(3)*X_plane+n_1(2)/n_1(3)*Y_plane-dot(n_1,p_1)/n_1(3)),'facecolor','green','facealpha',0.2);