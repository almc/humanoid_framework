%% biot-savart init
loop.radius = 0.7;
loop.dl     = 0.2;
% loop.dl     = 0.05;
loop.N      = round(2*pi / loop.dl) + 1;
loop.off_x  = 0;
loop.off_y  = 0;

% angles      = linspace(0,  1.7*pi, loop.N);
% angles_rev  = linspace(0, -1.7*pi, loop.N);

angles      = linspace(0,  2*pi, loop.N);
angles_rev  = linspace(0, -2*pi, loop.N);

% angles      = linspace(0,  1.5*pi, loop.N);
% angles_rev  = linspace(0, -1.5*pi, loop.N);

%% bent
% loop.pos = [loop.radius/2*cos(pi*angles) + loop.off_x;
%             loop.radius*sin(angles)      + loop.off_y;
%             loop.radius*cos(angles)      + 2.5];

% loop.pos_rev = [loop.radius/2*cos(pi*angles_rev) + loop.off_x;
%                 loop.radius*sin(angles_rev)      + loop.off_y;
%                 loop.radius*cos(angles_rev)      + 2.5];

%% semi-bent
% unbent_region = 1:15;
% loop.pos(:,unbent_region) = [             0*(angles(unbent_region)) + loop.off_x;
%                            loop.radius*sin(angles(unbent_region))   + loop.off_x;
%                            loop.radius*cos(angles(unbent_region))   + 2.5];
%
% loop.pos_rev(:,unbent_region) = [           0*(angles_rev(unbent_region)) + loop.off_x;
%                                loop.radius*sin(angles_rev(unbent_region)) + loop.off_y;
%                                loop.radius*cos(angles_rev(unbent_region)) + 2.5];

%% straight
loop.pos = [             0*(angles) + loop.off_x;
            loop.radius*sin(angles) + loop.off_y;
            loop.radius*cos(angles) + 2.5];

loop.pos_rev = [             0*(angles_rev) + loop.off_x;
                loop.radius*sin(angles_rev) + loop.off_y;
                loop.radius*cos(angles_rev) + 2.5];


loop.bound = [min(loop.pos(1,:)) - 0.5, max(loop.pos(1,:)) + 0.5;
              min(loop.pos(2,:)) - 0.5, max(loop.pos(2,:)) + 0.5;
              min(loop.pos(3,:)) - 0.5, max(loop.pos(3,:)) + 0.5];

%% loop variables
loop.span  = abs(loop.bound(:,2) - loop.bound(:,1));

scaling_axis = find(loop.span < 1.2);

loop.bound(scaling_axis,1) = loop.bound(scaling_axis,1) - 1;
loop.bound(scaling_axis,2) = loop.bound(scaling_axis,2) + 1;


loop.grd = gradient(loop.pos);

loop.perp_vector = [0, 0, 0]';
loop.center = [sum(loop.pos(1,:))./length(loop.pos(1,:));
               sum(loop.pos(2,:))./length(loop.pos(2,:));
               sum(loop.pos(3,:))./length(loop.pos(3,:))];

vector_length = 1;

for i = 1:length(loop.grd(1,:))-1
    perp_vec_local = cross([loop.grd(1,i  ), loop.grd(2,i  ), loop.grd(3,i  )]', ...
                           [loop.grd(1,i+1), loop.grd(2,i+1), loop.grd(3,i+1)]');
    loop.perp_vector = loop.perp_vector + perp_vec_local;
end

loop.parl_vector1 = loop.pos(:,1) - loop.center;
loop.parl_vector2 = cross(loop.perp_vector, loop.parl_vector1);

loop.perp_vector  = vector_length*-loop.perp_vector  ./ norm(loop.perp_vector ,2);
loop.parl_vector1 = vector_length*+loop.parl_vector1 ./ norm(loop.parl_vector1,2);
loop.parl_vector2 = vector_length*-loop.parl_vector2 ./ norm(loop.parl_vector2,2);


%% transformation between the world coordinate and the loop coordinate
a1 = [1,0,0]'; a2 = [0,1,0]'; a3 = [0,0,1]';
b1 = loop.parl_vector1; b2 = loop.parl_vector2; b3 = loop.perp_vector;
M  = [b1, b2, b3];
O  = [a1, a2, a3];
world_to_loop_3x3 = linsolve(M', O')';

t_loop = world_to_loop_3x3 * loop.center;
world_to_loop_4x4 = [world_to_loop_3x3(1,1) world_to_loop_3x3(1,2) world_to_loop_3x3(1,3) -t_loop(1);
                     world_to_loop_3x3(2,1) world_to_loop_3x3(2,2) world_to_loop_3x3(2,3) -t_loop(2);
                     world_to_loop_3x3(3,1) world_to_loop_3x3(3,2) world_to_loop_3x3(3,3) -t_loop(3);
                     0                      0                      0                       1       ];


%% grid #1
grid_step = 0.4;
[X_grid, Y_grid, Z_grid] = meshgrid(loop.bound(1,1):grid_step:loop.bound(1,2), ...
                                    loop.bound(2,1):grid_step:loop.bound(2,2), ....
                                    loop.bound(3,1):grid_step:loop.bound(3,2));
U_grid = zeros(size(X_grid)); V_grid = zeros(size(Y_grid)); W_grid = zeros(size(Z_grid));

for i = 1:length(loop.pos(1,:))-1
    pl = loop.pos(:,i  ); %[x(i); y(i); z(i)];
    dl = loop.pos(:,i+1) - loop.pos(:,i); %[x(i+1) - x(i); y(i+1) - y(i); z(i+1) - z(i)];
    for i1 = 1:size(X_grid,1)
        for i2 = 1:size(Y_grid,2)
            for i3 = 1:size(Z_grid,3)
                p  = [X_grid(i1,i2,i3); Y_grid(i1,i2,i3); Z_grid(i1,i2,i3)];
                r  = p - pl;
                if r < 0.25
                    continue
                end
                cp = cross(dl,r) ./ (norm(r)^3);
                U_grid(i1,i2,i3) = U_grid(i1,i2,i3) + cp(1);
                V_grid(i1,i2,i3) = V_grid(i1,i2,i3) + cp(2);
                W_grid(i1,i2,i3) = W_grid(i1,i2,i3) + cp(3);
            end
        end
    end
end
%
% %% grid #2
% X_grid2 = X_grid + 0.2*U_grid;
% Y_grid2 = Y_grid + 0.2*V_grid;
% Z_grid2 = Z_grid + 0.2*W_grid;
% U_grid2 = zeros(size(X_grid2)); V_grid2 = zeros(size(Y_grid2)); W_grid2 = zeros(size(Z_grid2));
%
% for i = 1:length(loop.pos(1,:))-1
%     pl = loop.pos(:,i  ); %[x(i); y(i); z(i)];
%     dl = loop.pos(:,i+1) - loop.pos(:,i); %[x(i+1) - x(i); y(i+1) - y(i); z(i+1) - z(i)];
%     for i1 = 1:size(X_grid2,1)
%         for i2 = 1:size(Y_grid2,2)
%             for i3 = 1:size(Z_grid2,3)
%                 p  = [X_grid2(i1,i2,i3); Y_grid2(i1,i2,i3); Z_grid2(i1,i2,i3)];
%                 r  = p - pl;
%                 if r < 0.25
%                     continue
%                 end
%                 cp = cross(dl,r) ./ (norm(r)^3);
%                 U_grid2(i1,i2,i3) = U_grid2(i1,i2,i3) + cp(1);
%                 V_grid2(i1,i2,i3) = V_grid2(i1,i2,i3) + cp(2);
%                 W_grid2(i1,i2,i3) = W_grid2(i1,i2,i3) + cp(3);
%             end
%         end
%     end
% end
%
% %% grid #3
% X_grid3 = X_grid2 + 0.2*U_grid2;
% Y_grid3 = Y_grid2 + 0.2*V_grid2;
% Z_grid3 = Z_grid2 + 0.2*W_grid2;
% U_grid3 = zeros(size(X_grid3)); V_grid3 = zeros(size(Y_grid3)); W_grid3 = zeros(size(Z_grid3));
%
% for i = 1:length(loop.pos(1,:))-1
%     pl = loop.pos(:,i  ); %[x(i); y(i); z(i)];
%     dl = loop.pos(:,i+1) - loop.pos(:,i); %[x(i+1) - x(i); y(i+1) - y(i); z(i+1) - z(i)];
%     for i1 = 1:size(X_grid3,1)
%         for i2 = 1:size(Y_grid3,2)
%             for i3 = 1:size(Z_grid3,3)
%                 p  = [X_grid3(i1,i2,i3); Y_grid3(i1,i2,i3); Z_grid3(i1,i2,i3)];
%                 r  = p - pl;
%                 if r < 0.25
%                     continue
%                 end
%                 cp = cross(dl,r) ./ (norm(r)^3);
%                 U_grid3(i1,i2,i3) = U_grid3(i1,i2,i3) + cp(1);
%                 V_grid3(i1,i2,i3) = V_grid3(i1,i2,i3) + cp(2);
%                 W_grid3(i1,i2,i3) = W_grid3(i1,i2,i3) + cp(3);
%             end
%         end
%     end
% end
%
% %% grid #4
% X_grid4 = X_grid3 + 0.2*U_grid3;
% Y_grid4 = Y_grid3 + 0.2*V_grid3;
% Z_grid4 = Z_grid3 + 0.2*W_grid3;
% U_grid4 = zeros(size(X_grid4)); V_grid4 = zeros(size(Y_grid4)); W_grid4 = zeros(size(Z_grid4));
%
% for i = 1:length(loop.pos(1,:))-1
%     pl = loop.pos(:,i  ); %[x(i); y(i); z(i)];
%     dl = loop.pos(:,i+1) - loop.pos(:,i); %[x(i+1) - x(i); y(i+1) - y(i); z(i+1) - z(i)];
%     for i1 = 1:size(X_grid4,1)
%         for i2 = 1:size(Y_grid4,2)
%             for i3 = 1:size(Z_grid4,3)
%                 p  = [X_grid4(i1,i2,i3); Y_grid4(i1,i2,i3); Z_grid4(i1,i2,i3)];
%                 r  = p - pl;
%                 if r < 0.25
%                     continue
%                 end
%                 cp = cross(dl,r) ./ (norm(r)^3);
%                 U_grid4(i1,i2,i3) = U_grid4(i1,i2,i3) + cp(1);
%                 V_grid4(i1,i2,i3) = V_grid4(i1,i2,i3) + cp(2);
%                 W_grid4(i1,i2,i3) = W_grid4(i1,i2,i3) + cp(3);
%             end
%         end
%     end
% end


%% move following the biot_savart law
biot_savart.alfa = 2;
biot_savart.beta = 1;

%% move perpendicular to the loop
% biot_savart.alfa = 0;
% biot_savart.beta = 2;

%% move parallel to the loop
% biot_savart.alfa = 2;
% biot_savart.beta = 0;

% biot_savart.alfa = 2;
% biot_savart.beta = 1.5;


frame = 1;
