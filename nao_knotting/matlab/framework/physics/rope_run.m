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
% [row, col] = find(coll_rope_rope < thr_rope_rope_long_comp);
% force_coll_rope_rope_long_comp = zeros(3, rope.N);
% for idx = 1:length(row)
%     if (row(idx) == col(idx)) % discard self point collision
%         continue
%     else
% %             disp('rope/rope long compression')
%         force_coll_rope_rope_long_comp(:,col(idx)) = k_repulsion_rope_long_comp * ...
%             (rope.pos(:,col(idx)) - rope.pos(:,row(idx)) ) / ...
%             (1 + coll_rope_rope(row(idx), col(idx)) );
%         force_coll_rope_rope_long_comp(:,row(idx)) = -force_coll_rope_rope_long_comp(:,col(idx));
%     end
% end

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

%% rope/rope looping point
if ~loop_strangled
    [row, col] = find(coll_rope_rope < 1.1*thr_rope_rope_trav_cros);
    rope_rope_looping_point = zeros(1, rope.N);
    min_dist_row = 0; min_dist_col = 0; min_dist_value = Inf;
    rope_loop_idx = [];
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
            rope_loop_idx = linspace(md1, md2, md2-md1+1);
            rope_rope_looping_point(rope_loop_idx) = 1;
        end
    end
    %     rope_rope_looping_point(min_dist_row) = 1;
    %     rope_rope_looping_point(min_dist_col) = 1;
end

%% force total
force_total   = 1*force_gravity + 1*force_spring_m + ...
    1*force_coll_loop_rope + ...
...%     1*force_coll_rope_rope_long_comp + ...
...%         0*force_coll_rope_rope_long_exte + ...
    1*force_coll_rope_rope_trav_cros;% + ...
%     1*force_floor;

%% velocity update due to force
rope.vel = rope.vel + (force_total ./ repmat(rope.dm,3,1) * dt);

%% setting trajectory of grasped points
%     rope.pos(:,end) = arm1.ref(:,n);
%     rope.vel(:,1) = rope.vel(:,1) + k_grasp*(arm1.ref(:,n) - rope.pos(:,1));

end_effectors = [arm1.ef, arm2.ef];

[arm_row, rope_col] = find(grasp_id_rope == 1);

rope.vel(:,rope_col) = rope.vel(:,rope_col) + k_grasp * ...
    (end_effectors(:,arm_row) - rope.pos(:,rope_col));

%% rope loop strangled stop motion
if ~isempty(rope_loop_idx)
%     disp('rope_loop')
    strgl_loop_rope = pdist2(loop.pos', rope.pos(:,rope_loop_idx)', 'euclidean');
    [row, col] = find( strgl_loop_rope == min(min(strgl_loop_rope)) );
    maximo = max  (strgl_loop_rope(row(1),:))
    promed = mean (strgl_loop_rope(row(1),:))
    if state >= 5.5 % trying to strangle loop
        if (max (strgl_loop_rope(row(1),:)) < 1.0*thr_rope_loop_strangled ...
         && mean(strgl_loop_rope(row(1),:)) < 0.9*thr_rope_loop_strangled ) || loop_strangled
    %        disp('strangled_loop')
           rope.vel(:,rope_loop_idx) = 0.0;
           %% rope strangled creation for biot_savart law
           if loop_strangled == 0
               rope_strangled;
           end
           loop_strangled = 1;
        end
    end
%        rope_vel(:,rope_loop) = zeros(3,length(rope_loop)); %rope_vel(:,rope_loop(end)) = [0,0,0]';
%        rope.vel(:,:) = zeros(size(rope.vel));
end


rope.vel(rope.vel >  0.9) =  0.9;
rope.vel(rope.vel < -0.9) = -0.9;
    
%% position update due to velocity
rope.pos = rope.pos + rope.vel * dt;


%% stiff springs hack
[row_c, col_c] = find(coll_rope_rope < 0.95*rope.dl);
[row_e, col_e] = find(coll_rope_rope > 1.05*rope.dl);
for idx = 1:length(row_c)
   if any(row_c(idx) == rope_loop_idx) || any(col_c(idx) == rope_loop_idx)
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
   if any(row_e(idx) == rope_loop_idx) || any(col_e(idx) == rope_loop_idx)
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

