%% grid #1

% area.bound = [min(loop.pos(1,:)) - 3, max(loop.pos(1,:)) + 3;
%               min(loop.pos(2,:)) - 3, max(loop.pos(2,:)) + 3;
%               min(loop.pos(3,:)) - 3, max(loop.pos(3,:)) + 3];
          
area.bound = [min(loop.pos(1,:)) - 4, max(loop.pos(1,:)) + 4;
              min(loop.pos(2,:)) - 4, max(loop.pos(2,:)) + 4;
              min(loop.pos(3,:)) - 3, max(loop.pos(3,:)) + 5];

grid_step = 1.0;

[Xa_grid, Ya_grid, Za_grid] = meshgrid(area.bound(1,1):grid_step:area.bound(1,2), ...
                                       area.bound(2,1):grid_step:area.bound(2,2), ....
                                       area.bound(3,1):grid_step:area.bound(3,2));

Ua_grid = zeros(size(Xa_grid)); % V_grid = zeros(size(Y_grid)); W_grid = zeros(size(Z_grid));

for i1 = 1:size(Xa_grid,1)
    for i2 = 1:size(Ya_grid,2)
        for i3 = 1:size(Za_grid,3)
            p = [Xa_grid(i1,i2,i3); Ya_grid(i1,i2,i3); Za_grid(i1,i2,i3)];
            %Ua_grid(i1,i2,i3) = arm_range(p, base.pos_rot(:), arm1);
            Ua_grid(i1,i2,i3) = arm_range(p, arm2.j1, arm2);
        end
    end
end