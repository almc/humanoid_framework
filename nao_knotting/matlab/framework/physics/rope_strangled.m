%% rope strangled

rope_loop.pos     = rope.pos(:,rope_loop_idx);
rope_loop.pos_rev = fliplr(rope_loop.pos);
        
rope_loop.bound = [min(rope_loop.pos(1,:)) - 0.5, max(rope_loop.pos(1,:)) + 0.5;
                   min(rope_loop.pos(2,:)) - 0.5, max(rope_loop.pos(2,:)) + 0.5;
                   min(rope_loop.pos(3,:)) - 0.5, max(rope_loop.pos(3,:)) + 0.5];

rope_loop.span  = abs(rope_loop.bound(:,2) - rope_loop.bound(:,1));

scaling_axis    = find(rope_loop.span < 1.2);

rope_loop.bound(scaling_axis,1) = rope_loop.bound(scaling_axis,1) - 1;
rope_loop.bound(scaling_axis,2) = rope_loop.bound(scaling_axis,2) + 1;

rope_loop.grd = gradient(rope_loop.pos);

rope_loop.perp_vector = [0, 0, 0]';
rope_loop.center = [sum(rope_loop.pos(1,:)) ./ length(rope_loop.pos(1,:));
                    sum(rope_loop.pos(2,:)) ./ length(rope_loop.pos(2,:));
                    sum(rope_loop.pos(3,:)) ./ length(rope_loop.pos(3,:))];

vector_length = 1;

for i = 1:length(rope_loop.grd(1,:))-1
    perp_vec_local = cross([rope_loop.grd(1,i  ), rope_loop.grd(2,i  ), rope_loop.grd(3,i  )]', ...
                           [rope_loop.grd(1,i+1), rope_loop.grd(2,i+1), rope_loop.grd(3,i+1)]');
    rope_loop.perp_vector = rope_loop.perp_vector + perp_vec_local;
end

rope_loop.parl_vector1 = rope_loop.pos(:,1) - rope_loop.center;
rope_loop.parl_vector2 = cross(rope_loop.perp_vector, rope_loop.parl_vector1);

dist_center_rope_loop = zeros(1,size(rope_loop.pos,2));
% rope_loop.pos(:,:) - repmat(rope_loop.center,[1,size(rope_loop.pos,2)])
for i = 1:size(rope_loop.pos,2)
    dist_center_rope_loop(i) = norm(rope_loop.pos(:,i) - rope_loop.center,2);
end
rope_loop.radius = min(dist_center_rope_loop);

rope_loop.perp_vector  = vector_length*-rope_loop.perp_vector  ./ norm(rope_loop.perp_vector ,2);
rope_loop.parl_vector1 = vector_length*+rope_loop.parl_vector1 ./ norm(rope_loop.parl_vector1,2);            
rope_loop.parl_vector2 = vector_length*-rope_loop.parl_vector2 ./ norm(rope_loop.parl_vector2,2);


%% transformation between the world coordinate and the rope_loop coordinate
ra1 = [1,0,0]'; ra2 = [0,1,0]'; ra3 = [0,0,1]';
rb1 = rope_loop.parl_vector1; rb2 = rope_loop.parl_vector2; rb3 = rope_loop.perp_vector;
rM  = [rb1, rb2, rb3];
rO  = [ra1, ra2, ra3];
world_to_rope_loop_3x3 = linsolve(rM, rO);

t_rope_loop = world_to_rope_loop_3x3 * rope_loop.center;
world_to_rope_loop_4x4 = [world_to_rope_loop_3x3(1,1) world_to_rope_loop_3x3(1,2) world_to_rope_loop_3x3(1,3) -t_rope_loop(1);
                          world_to_rope_loop_3x3(2,1) world_to_rope_loop_3x3(2,2) world_to_rope_loop_3x3(2,3) -t_rope_loop(2);
                          world_to_rope_loop_3x3(3,1) world_to_rope_loop_3x3(3,2) world_to_rope_loop_3x3(3,3) -t_rope_loop(3);
                          0                           0                           0                            1            ];



%% modifying rope constants to permit knotting
% k_repulsion_loop = 0.01;
% k_repulsion_rope_long_comp = 0.0;
% k_repulsion_rope_long_exte = 0.0;
% k_repulsion_rope_trav_cros = 0.020;
% 
% thr_loop_rope = 0.1; % 6.0 * rope.dl; % depends on the discretization of the loop
% thr_rope_rope_long_comp =  0.0 * rope.dl; % not less than 0.5, not more than 1.0
% thr_rope_rope_long_exte =  0.0 * rope.dl; % stretch limit of the rope 1.5*dl
% thr_rope_rope_trav_cros = 0.05; % 3.0 * rope.dl; % distance to prevent self crossing % 2.5 and 10 units
% thr_rope_loop_strangled = 0.60; % 4.0 * rope.dl;


%% grid #1
grid_step = 0.4;
[rX_grid, rY_grid, rZ_grid] = meshgrid(rope_loop.bound(1,1):grid_step:rope_loop.bound(1,2), ...
                                       rope_loop.bound(2,1):grid_step:rope_loop.bound(2,2), ....
                                       rope_loop.bound(3,1):grid_step:rope_loop.bound(3,2));
rU_grid = zeros(size(rX_grid)); rV_grid = zeros(size(rY_grid)); rW_grid = zeros(size(rZ_grid));

for i = 1:length(rope_loop.pos(1,:))-1
    pl = rope_loop.pos(:,i  ); %[x(i); y(i); z(i)];
    dl = rope_loop.pos(:,i+1) - rope_loop.pos(:,i); %[x(i+1) - x(i); y(i+1) - y(i); z(i+1) - z(i)];
    for i1 = 1:size(rX_grid,1)
        for i2 = 1:size(rY_grid,2)
            for i3 = 1:size(rZ_grid,3)
                p  = [rX_grid(i1,i2,i3); rY_grid(i1,i2,i3); rZ_grid(i1,i2,i3)];
                r  = p - pl;
                if r < 0.25
                    continue
                end
                cp = cross(dl,r) ./ (norm(r)^3);
                rU_grid(i1,i2,i3) = rU_grid(i1,i2,i3) + cp(1);
                rV_grid(i1,i2,i3) = rV_grid(i1,i2,i3) + cp(2);
                rW_grid(i1,i2,i3) = rW_grid(i1,i2,i3) + cp(3);
            end
        end
    end
end