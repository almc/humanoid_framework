%% plotting loop
plot3(loop.pos(1,:), loop.pos(2,:), loop.pos(3,:), 'b', 'LineWidth', 4);
plot3([loop.pos(1,end), loop.pos(1,1)], ...
      [loop.pos(2,end), loop.pos(2,1)], ...
      [loop.pos(3,end), loop.pos(3,1)], 'r-.', 'LineWidth', 2); % virtual link

%% plotting center of the loop
plot3(loop.center(1), loop.center(2), loop.center(3), 'ko','MarkerFaceColor','c');

%% plotting parallel and perpendicular loop vector
quiver3(loop.center(1)      , loop.center(2)      , loop.center(3)      , ...
        loop.parl_vector1(1), loop.parl_vector1(2), loop.parl_vector1(3), 'r');

quiver3(loop.center(1)      , loop.center(2)      , loop.center(3)      , ...
        loop.parl_vector2(1), loop.parl_vector2(2), loop.parl_vector2(3), 'g');

quiver3(loop.center(1)      , loop.center(2)      , loop.center(3)      , ...
        loop.perp_vector(1) , loop.perp_vector(2) , loop.perp_vector(3) , 'b');

if loop_strangled && state == 7
    %% plotting rope_loop axes
    quiver3(rope_loop.center(1)      , rope_loop.center(2)      , rope_loop.center(3)      , ...
            rope_loop.parl_vector2(1), rope_loop.parl_vector2(2), rope_loop.parl_vector2(3), 'r');

    quiver3(rope_loop.center(1)      , rope_loop.center(2)      , rope_loop.center(3)      , ...
            rope_loop.perp_vector(1) , rope_loop.perp_vector(2) , rope_loop.perp_vector(3) , 'g');

    quiver3(rope_loop.center(1)      , rope_loop.center(2)      , rope_loop.center(3)      , ...
            rope_loop.parl_vector1(1), rope_loop.parl_vector1(2), rope_loop.parl_vector1(3), 'b');
    
    %% plotting rope_loop vectorfield
    quiver3(rX_grid,  rY_grid,  rZ_grid,  rU_grid,  rV_grid, rW_grid,  1, 'k');
end

if state == 2
    quiver3(X_grid,  Y_grid,  Z_grid,  U_grid,  V_grid,  W_grid,  1, 'k');
end
%persistent frame;
% if isempty(frame)
%   frame = 1;
% end

% if     frame == 1
%     quiver3(X_grid,  Y_grid,  Z_grid,  U_grid,  V_grid,  W_grid,  1, 'k');
%     frame = 2;
% elseif frame == 2
%     quiver3(X_grid2, Y_grid2, Z_grid2, U_grid2, V_grid2, W_grid2, 1, 'k');
%     frame = 3;
% elseif frame == 3
%     quiver3(X_grid3, Y_grid3, Z_grid3, U_grid3, V_grid3, W_grid3, 1, 'k');
%     frame = 4;
% elseif frame == 4
%     quiver3(X_grid4, Y_grid4, Z_grid4, U_grid4, V_grid4, W_grid4, 1, 'k');
%     frame = 1;
% end

% quiver3(arm1.ef(1), arm1.ef(2), arm1.ef(3), arm1.grad(1), arm1.grad(2), arm1.grad(3), 'r');
% quiver3(arm2.ef(1), arm2.ef(2), arm2.ef(3), arm2.grad(1), arm2.grad(2), arm2.grad(3), 'b'); 

%% debug plot
plot3(debug.point1(1,:),debug.point1(2,:),debug.point1(3,:),'ko','MarkerFaceColor','c', 'MarkerSize', 3);
plot3(debug.point2(1,:),debug.point2(2,:),debug.point2(3,:),'ko','MarkerFaceColor','m', 'MarkerSize', 3);
plot3(debug.point3(1,:),debug.point3(2,:),debug.point3(3,:),'ko','MarkerFaceColor','g', 'MarkerSize', 3);


xl = 0.5;
yl = 0.5;
patch_x = [-xl, xl,  xl, -xl];
patch_y = [ yl, yl, -yl, -yl];


if calc_reachability == 1
    %% plot arm reachability
    for i1 = 1:size(Xa_grid,1)
        for i2 = 1:size(Ya_grid,2)
            for i3 = 1:size(Za_grid,3)
                p = [Xa_grid(i1,i2,i3); Ya_grid(i1,i2,i3); Za_grid(i1,i2,i3)];
                if Ua_grid(i1,i2,i3) == 1
%                     plot3(p(1),p(2),p(3),'ko','MarkerFaceColor','g', 'MarkerSize', 5)
                    fill3(patch_x + p(1), patch_y + p(2), 0*patch_y + p(3), 'g', 'FaceAlpha', 0.1, 'EdgeColor', 'g', 'EdgeAlpha', 0.1);
                else
%                     plot3(p(1),p(2),p(3),'ko','MarkerFaceColor','r', 'MarkerSize', 5)
                    fill3(patch_x + p(1), patch_y + p(2), 0*patch_y + p(3), 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'r', 'EdgeAlpha', 0.1);
                end
            end
        end
    end
end