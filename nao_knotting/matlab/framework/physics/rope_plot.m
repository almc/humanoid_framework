%% drawing rope
plot3(rope.pos(1,:), rope.pos(2,:), rope.pos(3,:), 'Color', 'r', 'LineWidth', 2);
% plot3(rope.pos(1,:), rope.pos(2,:), rope.pos(3,:), 'Color', 'k', 'LineWidth', 0.5);
% plot3(rope.pos(1,:), rope.pos(2,:), rope.pos(3,:), 'ko'  ,'MarkerFaceColor','r', 'MarkerSize', 4);

%% drawing rope loop
plot3(rope.pos(1,rope_loop_idx), rope.pos(2,rope_loop_idx), rope.pos(3,rope_loop_idx), 'Color', 'k', 'LineWidth', 1);
% plot3(rope.pos(1,rope_loop_idx), rope.pos(2,rope_loop_idx), rope.pos(3,rope_loop_idx), 'ko', 'MarkerFaceColor', 'k');