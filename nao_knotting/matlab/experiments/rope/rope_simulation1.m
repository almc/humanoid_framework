%% rope simulation

%% initialization
clear all, close all, clc

%% time variables
T    = 100;
dt   = 0.1;
N    = round(T / dt) + 1;
time = linspace(0, T, N);

%% rope variables
% rope.dl     = 0.05;
rope.L      = 2.0;
% rope.N      = round(rope.L / rope.dl) + 1;
rope.N      = 30;
rope.E      = rope.N - 1;
rope.dl     = rope.L / rope.E;
rope.q      = linspace(0, rope.L, rope.N);
rope.M      = 0.01;
rope.dm     = rope.M / rope.N * ones(1,rope.N);

% rope.pos    = [1.0*rope.q - 4;
%                0.0*rope.q + 0;
%                0.0*rope.q + 0];

rope.pos    = [+1.0*rope.q - 4;
               +0.0*rope.q + 0;
               +0.0*rope.q + 1];

rope.vel    = [0*rope.q;
               0*rope.q;
               0*rope.q];

%rope.edges  = gradient(rope.pos);

g = [  0,  0, -0.01];
k_spring_compression =   0.002;
k_spring_friction    =  0.0001;
k_floor_friction     =   0.0;

%% loop variables
loop.radius = 0.7;
loop.dl     = 0.2;
loop.N      = round(2*pi / loop.dl) + 1;
angles      = linspace(0,  2*pi, loop.N);
% angles_rev  = linspace(0, -2*pi, loop.N);


loop.pos = [             0*(angles);
            loop.radius*sin(angles);
            loop.radius*cos(angles) + 1];

% loop.radius = 0.7;
% loop.dl     = rope.dl; % 0.2;
% loop.N      = round(2*pi / loop.dl) + 1;
% angles      = linspace(0, 2*pi, loop.N);
%
% loop.pos = [             0*(angles);
%                loop.radius*sin(angles);
%                loop.radius*cos(angles) + 1];

%% trajectory generation
r = 1;
angle  = linspace(0, 2*pi, N/4);
angle2 = linspace(pi,pi/3, N/4);
angle3 = linspace(0, 3*pi/4, N/4);
linsp  = linspace(-1,2,N/4);
linsp2 = linspace( 0,1,N/4);
linsp3 = linspace( 1,0.6,N/8);
linsp4 = linspace(-0.7071, 0.05, N/8);
linsp5 = linspace( 0.7071, 0.35, N/8);
linsp6 = linspace(0.6, 1.2, N/4);

x = [  linsp,       2*cos(angle3), 2*linsp4                      , 0*linsp6 + 2*linsp4(end);
     0*linsp-2,     2*cos(angle2), 0*linsp4 + 2*cos(angle2(end)) , 0*linsp6 + 2*cos(angle2(end))];
y = [0*linsp,       2*sin(angle3), 2*linsp5                      , 0*linsp6 + 2*linsp5(end);
     0*linsp+0,     2*sin(angle2), 0*linsp5 + 2*sin(angle2(end)) , 0*linsp6 + 2*sin(angle2(end))];
z = [1 + 0*linsp,   1 + 0*linsp  , linsp3                        , 1*linsp6;
     0*linsp+0,            linsp2, 0*linsp3 + linsp2(end)        , 0*linsp6 + linsp2(end)];

k_grasp = 10;
k_hold  = 0;
%% collision variables
coll_loop_rope = zeros(loop.N, rope.N);
coll_rope_rope = zeros(rope.N, rope.N);

n_points_discarded = 5;
k_repulsion_loop = 0.020;
k_repulsion_rope_long_comp = 0.0;
k_repulsion_rope_long_exte = 0;
k_repulsion_rope_trav_cros = 0.040;

thr_loop_rope = 0.2; % 6.0 * rope.dl; % depends on the discretization of the loop
thr_rope_rope_long_comp =  0.0 * rope.dl; % not less than 0.5, not more than 1.0
thr_rope_rope_long_exte =  0.0 * rope.dl; % stretch limit of the rope 1.5*dl
thr_rope_rope_trav_cros = 0.1; % 3.0 * rope.dl; % distance to prevent self crossing % 2.5 and 10 units
thr_rope_loop_strangled = 0.45; % 4.0 * rope.dl; % distance to say the rope loop is fixed

% k_repulsion_loop = 0.020;
% k_repulsion_rope_long_comp = 0.0;
% k_repulsion_rope_long_exte = 0;
% k_repulsion_rope_trav_cros = 0;
%
% thr_loop_rope = 4.0 * rope.dl; % depends on the discretization of the loop
% thr_rope_rope_long_comp =  0.0 * rope.dl; % not less than 0.5, not more than 1.0
% thr_rope_rope_long_exte =  0.0 * rope.dl; % stretch limit of the rope 1.5*dl
% thr_rope_rope_trav_cros =  3.0 * rope.dl; % distance to prevent self crossing % 2.5 and 10 units
% thr_rope_loop_strangled = 4.0 * rope.dl; % distance to say the rope loop is fixed

n_arms = 2;
grasp_id_rope = zeros(n_arms, rope.N);
grasp_id_rope(2,1)   = 1;
grasp_id_rope(1,end) = 1;
rope_loop = [];
loop_strangled = 0;

%% main loop
for n = 1:N
    time(n);

    %% gravity
    force_gravity = g' * rope.dm;
    %% spring and friction
    force_spring   = zeros(3, rope.E);
    force_spring_m = zeros(3, rope.N);
    for s = 1:rope.E
        spring_vector = rope.pos(:,s) - rope.pos(:,s+1);
        r = norm(spring_vector, 2);
        if (r ~= 0)
            force_spring(:,s) = (spring_vector / r) * (r - rope.dl) * (-k_spring_compression);
        else
            disp('error')
        end
        force_spring(:,s) = force_spring(:,s) - (rope.vel(:,s) - rope.vel(:,s+1)) * k_spring_friction;
        force_spring_m(:,s)   = force_spring_m(:,s)   + force_spring(:,s);
        force_spring_m(:,s+1) = force_spring_m(:,s+1) - force_spring(:,s);
    end

    force_spring_m_lim = 3;
    force_spring_m(force_spring_m >  force_spring_m_lim) =  force_spring_m_lim;
    force_spring_m(force_spring_m < -force_spring_m_lim) = -force_spring_m_lim;
%     max(max(force_spring_m))

    %% loop/rope collisions
    coll_loop_rope = pdist2(loop.pos', rope.pos', 'euclidean');
    coll_rope_rope = pdist2(rope.pos', rope.pos', 'euclidean');

    [row, col] = find(coll_loop_rope < thr_loop_rope);
    force_coll_loop_rope = zeros(3, rope.N);
    for idx = 1:length(row)
%             disp('loop/rope collision')
            force_coll_loop_rope(:,col(idx)) = k_repulsion_loop * ...
                (rope.pos(:,col(idx)) - loop.pos(:,row(idx)) ) / ...
                (1 + coll_loop_rope(row(idx), col(idx)) );
    end
    %% rope/rope collisions (longitudinal compression)
%     [row, col] = find(coll_rope_rope < thr_rope_rope_long_comp);
%     force_coll_rope_rope_long_comp = zeros(3, rope.N);
%     for idx = 1:length(row)
%         if (row(idx) == col(idx)) % discard self point collision
%             continue
%         else
% %             disp('rope/rope long compression')
%             force_coll_rope_rope_long_comp(:,col(idx)) = k_repulsion_rope_long_comp * ...
%                 (rope.pos(:,col(idx)) - rope.pos(:,row(idx)) ) / ...
%                 (1 + coll_rope_rope(row(idx), col(idx)) );
%             force_coll_rope_rope_long_comp(:,row(idx)) = -force_coll_rope_rope_long_comp(:,col(idx));
%         end
%     end




    %% rope/rope collisions (longitudinal extension)
%     [row, col] = find(coll_rope_rope > thr_rope_rope_long_exte);
%     force_coll_rope_rope_long_exte = zeros(3, rope.N);
%     for idx = 1:length(row)
%         if ( abs(row(idx) - col(idx)) ~= 1) % discard not neighboring points
%             continue
%         else
% %             disp('rope/rope long extension')
%             force_coll_rope_rope_long_exte(:,col(idx)) = k_repulsion_rope_long_exte * ...
%                 (rope.pos(:,col(idx)) - rope.pos(:,row(idx)) ) / ...
%                 (1 + coll_rope_rope(row(idx), col(idx)) );
%             force_coll_rope_rope_long_exte(:,row(idx)) = -force_coll_rope_rope_long_exte(:,col(idx));
%         end
%     end

    %% rope/rope point based collisions (travelsal crossing)
    [row, col] = find(coll_rope_rope < thr_rope_rope_trav_cros);
    force_coll_rope_rope_trav_cros = zeros(3, rope.N);
    for idx = 1:length(row)
        if ( abs(row(idx) - col(idx)) < n_points_discarded) % discard points too close to care
            continue
        else
%             disp('rope/rope traversal cross')
            force_coll_rope_rope_trav_cros(:,col(idx)) = k_repulsion_rope_trav_cros * ...
                (rope.pos(:,col(idx)) - rope.pos(:,row(idx)) ) / ...
                (1 + coll_rope_rope(row(idx), col(idx)) );
            force_coll_rope_rope_trav_cros(:,row(idx)) = -force_coll_rope_rope_trav_cros(:,col(idx));
        end
    end

%     %% rope/rope line based collisions (traversal crossing)
%     [row, col] = find(coll_rope_rope < thr_rope_rope_trav_cros);
%     force_coll_rope_rope_trav_cros = zeros(3, rope.N);
%     for idx = 1:length(row)
%          if ( abs(row(idx) - col(idx)) < n_points_discarded) % discard points too close to care
%             continue
%          else
%             line1_a = [rope.pos(:,row(idx))'; rope.pos(:,row(idx)+1)'];
%             line1_b = [rope.pos(:,row(idx))'; rope.pos(:,row(idx)-1)'];
%
%             line2_a = [rope.pos(:,col(idx))'; rope.pos(:,col(idx)+1)'];
%             line2_b = [rope.pos(:,col(idx))'; rope.pos(:,col(idx)-1)'];
%
%             [d_1a2a, p_1a2a, q_1a2a] = dist_between_lines(line1_a, line2_a);
%             [d_1a2b, p_1a2b, q_1a2b] = dist_between_lines(line1_a, line2_b);
%             [d_1b2a, p_1b2a, q_1b2a] = dist_between_lines(line1_b, line2_a);
%             [d_1b2b, p_1b2b, q_1b2b] = dist_between_lines(line1_a, line2_a);
%          end
%     end



    %% rope/rope looping point
    if ~loop_strangled
    [row, col] = find(coll_rope_rope < 1.1*thr_rope_rope_trav_cros);
    rope_rope_looping_point = zeros(1, rope.N);
    min_dist_row = 0; min_dist_col = 0; min_dist_value = Inf;
    rope_loop = [];
    for idx = 1:length(row)
        if ( abs(row(idx) - col(idx)) < n_points_discarded) % discard points too close to care
            continue
        else
            if min_dist_value >= coll_rope_rope(row(idx), col(idx))
%                 disp('found possible intersection point')
                min_dist_value = coll_rope_rope(row(idx), col(idx));
                min_dist_row = row(idx); min_dist_col = col(idx);
            end
        end
    end
    if (min_dist_row ~= 0 && min_dist_col ~= 0)
        md1 = min([min_dist_row, min_dist_col]);
        md2 = max([min_dist_row, min_dist_col]);
        %[md1, md2] = sort([min_dist_row, min_dist_col],2)
        if (md1 < md2)
            rope_loop = linspace(md1, md2, md2-md1+1);
            rope_rope_looping_point(rope_loop) = 1;
        end
    end
%     rope_rope_looping_point(min_dist_row) = 1;
%     rope_rope_looping_point(min_dist_col) = 1;
    end


%     %% floor floor friction
%     force_floor = zeros(3, rope.N);
% %     force_floor(:, col_idx) = repmat(rope.dm(:,col_idx),3,1) .* rope.vel(:,col_idx) * k_floor_friction;
%     force_floor(1, col_idx) = rope.dm(1,col_idx) .* rope.vel(1,col_idx) * k_floor_friction;
%     force_floor(2, col_idx) = rope.dm(1,col_idx) .* rope.vel(2,col_idx) * k_floor_friction;
%
%     if loop_strangled
%        force_coll_loop_rope = zeros(size(force_coll_loop_rope));
%     end
%
    %% force total
    force_total   = 1*force_gravity + 1*force_spring_m + ...
        1*force_coll_loop_rope + ...
...%         1*force_coll_rope_rope_long_comp + ...
...%         0*force_coll_rope_rope_long_exte + ...
        1*force_coll_rope_rope_trav_cros; % + ...
%         1*force_floor;

    %% velocity update due to force
    rope.vel = rope.vel + (force_total ./ repmat(rope.dm,3,1) * dt);

    %% setting trajectory of grasped points
%     rope.pos(:,end) = [x(n), y(n), z(n)];
%     rope.vel(:,1) = rope.vel(:,1) + k_grasp*([x(n),y(n),z(n)]' - rope.pos(:,1));
    [arm_row, rope_col] = find(grasp_id_rope == 1);
    rope.vel(:,rope_col) = rope.vel(:,rope_col) + k_grasp * ...
        ([x(arm_row,n), y(arm_row,n),z(arm_row,n)]' - rope.pos(:,rope_col));

    %% rope loop strangled stop motion
    if ~isempty(rope_loop)
        disp('rope_loop')
        strgl_loop_rope = pdist2(loop.pos', rope.pos(:,rope_loop)', 'euclidean');
        [row, col] = find( strgl_loop_rope == min(min(strgl_loop_rope)) );
        if max(strgl_loop_rope(row(1),:)) < thr_rope_loop_strangled || loop_strangled
           disp('strangled_loop')
           loop_strangled = 1;
           rope.vel(:,rope_loop) = 0.0;
        end
    end

    %% air friction viscosity
%     for s = 1:rope.E
%         spring_vector = rope.pos(:,s) - rope.pos(:,s+1);
%         r = norm(spring_vector, 2);
%         if (r ~= 0)
%             force_spring(:,s) = (spring_vector / r) * (r - rope.dl) * (-k_spring_compression);
%         else
%             disp('error')
%         end
%         force_spring(:,s) = force_spring(:,s) - (rope.vel(:,s) - rope.vel(:,s+1)) * k_spring_friction;
%         force_spring_m(:,s)   = force_spring_m(:,s)   + force_spring(:,s);
%         force_spring_m(:,s+1) = force_spring_m(:,s+1) - force_spring(:,s);
%     end

    rope.vel(rope.vel >  0.5) =  0.5;
    rope.vel(rope.vel < -0.5) = -0.5;

    %% position update due to velocity
    rope.pos = rope.pos + 1*rope.vel * dt;

    %% stiff springs hack
    [row_c, col_c] = find(coll_rope_rope < 0.95*rope.dl);
    [row_e, col_e] = find(coll_rope_rope > 1.05*rope.dl);
    for idx = 1:length(row_c)
       if any(row_c(idx) == rope_loop) || any(col_c(idx) == rope_loop)
           continue
       end
       if abs(row_c(idx) - col_c(idx)) == 1 % they are connected
%            disp('compresion')
           dl_vector = rope.pos(:,col_c(idx)) - rope.pos(:,row_c(idx));
           dl_length = norm(dl_vector,2);
           compensation_c = rope.dl - dl_length;
           rope.pos(:,col_c(idx)) = rope.pos(:,col_c(idx)) + 0.2*compensation_c*dl_vector/dl_length;
           rope.pos(:,row_c(idx)) = rope.pos(:,row_c(idx)) - 0.2*compensation_c*dl_vector/dl_length;
       end
    end
    for idx = 1:length(row_e)
       if any(row_e(idx) == rope_loop) || any(col_e(idx) == rope_loop)
           continue
       end
       if abs(row_e(idx) - col_e(idx)) == 1 % they are connected
%            disp('expansion')
           dl_vector = rope.pos(:,col_e(idx)) - rope.pos(:,row_e(idx));
           dl_length = norm(dl_vector,2);
           compensation_e = dl_length - rope.dl;
           rope.pos(:,col_e(idx)) = rope.pos(:,col_e(idx)) - 0.2*compensation_e*dl_vector/dl_length;
           rope.pos(:,row_e(idx)) = rope.pos(:,row_e(idx)) + 0.2*compensation_e*dl_vector/dl_length;
       end
    end

    %% floor collisions
    col_idx = find(rope.pos(3,:) < 0);
    rope.pos(3, col_idx) = 0;

    [a1,b1] = view;

    %% drawing trajectory
    plot3(x(1,:), y(1,:), z(1,:) ,'g');
    title ('Rope Simulation');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    view([a1,b1]); axis([-2 +2, -2 +2, +0 +2]);
    hold on, grid on
    plot3(x(2,:), y(2,:), z(2,:) ,'y');
    plot3(x(1,n), y(1,n), z(1,n) ,'ko','MarkerFaceColor','g');
    plot3(x(2,n), y(2,n), z(2,n) ,'ko','MarkerFaceColor','y');

    %% drawing rope
    plot3(rope.pos(1,:), rope.pos(2,:), rope.pos(3,:), 'Color', 'r', 'LineWidth', 4);
    plot3(rope.pos(1,:), rope.pos(2,:), rope.pos(3,:), 'ko','MarkerFaceColor','r');

    %% drawing rope loop
    plot3(rope.pos(1,rope_loop), rope.pos(2,rope_loop), rope.pos(3,rope_loop), 'Color', 'k', 'LineWidth', 4);
    plot3(rope.pos(1,rope_loop), rope.pos(2,rope_loop), rope.pos(3,rope_loop), 'ko', 'MarkerFaceColor', 'k');

    %% drawing loop
    plot3(loop.pos(1,:), loop.pos(2,:), loop.pos(3,:), 'LineWidth', 10);
    plot3(loop.pos(1,:), loop.pos(2,:), loop.pos(3,:), 'ko','MarkerFaceColor','b');
    hold off

    %% wait draw and clear figure
    drawnow

end

close all
