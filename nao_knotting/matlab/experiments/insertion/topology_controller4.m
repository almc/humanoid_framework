clear all, close all

%% figure parameters
hFig = figure;
% set(hFig, 'Position', [500 2000 1600 1000])
view([1 1 1]);
axis([-2 2,-2 2, -2 2]);
% quiver3(x,y,z,u,v,w,scale)
hold on, grid on

%% loop original definition
radius = 1;
dif_theta = 0.1;
theta = 0:dif_theta:2*pi;
n_theta = size(theta,2);
xo = radius*cos(-theta);
yo = radius*sin(-theta);
zo = 0*theta;
% zo = radius/2*cos(4*pi*theta);
x = xo; y = yo; z = zo;

% calculate_loop_properties;
% visualize_loop;
    
%% precalculation of the magnetic field over the grid
% grid_step = 0.4;
% [X_grid,Y_grid,Z_grid] = meshgrid(-2:grid_step:2, -2:grid_step:2, -2:grid_step:2);
% U_grid = zeros(size(X_grid)); V_grid = zeros(size(Y_grid)); W_grid = zeros(size(Z_grid));
% for i = 1:length(x)-1
%     pl = [x(i); y(i); z(i)];
%     dl = [x(i+1) - x(i); y(i+1) - y(i); z(i+1) - z(i)];
%     for i1 = 1:size(X_grid,1)
%         for i2 = 1:size(Y_grid,2)
%             for i3 = 1:size(Z_grid,3)
%                 p = [X_grid(i1,i2,i3); Y_grid(i1,i2,i3); Z_grid(i1,i2,i3)];
%                 r  = p - pl;
%                 if r < 0.25
%                     continue
%                 end
%                 cp = cross(dl,r) ./ (norm(r)^3);
%                 U_grid(i1,i2,i3) = U_grid(i1,i2,i3) + cp(1);
%                 V_grid(i1,i2,i3) = V_grid(i1,i2,i3) + cp(2);
%                 W_grid(i1,i2,i3) = W_grid(i1,i2,i3) + cp(3);
%             end
%         end
%     end
% end
% quiver3(X_grid, Y_grid, Z_grid, U_grid, V_grid, W_grid, 1, 'k');

N = 100;
p_traj = zeros(3,N);
p  = [2; 0; 2];
% alfa = 1.5;
% beta = 0;
alfa = 1.0;
beta = 1;
gamma = 0.1;
record_movie = 1;
grad_final = zeros(3,1);

output_video = 0;
if output_video == 1
    filename = 'ex_insert.mp4';
    profile  = 'MPEG-4';
    writerObj = VideoWriter(filename);
    writerObj.FrameRate = 24;
    open(writerObj);
end

for n = 1:N
    %% hybrid parameters
%     if n == 20
%         beta = 0;
%         alfa = 1.5;
%     end
%     if n == 60
%         beta = 1.5;
%         alfa = 0;
%     end
    
    grad = [0; 0; 0];
    for i = 1:length(x)-1
        pl = [x(i); y(i); z(i)];
        dl = [x(i+1) - x(i); y(i+1) - y(i); z(i+1) - z(i)];
        rad  = p - pl;
        cp = cross(dl,rad) ./ (norm(rad)^3);
        grad = grad + cp;
    end
    %% virtual loop closing
    pl = [x(end); y(end); z(end)];
    dl = [x(1) - x(end); y(1) - y(end); z(1) - z(end)];
    rad  = p - pl;
    cp = cross(dl,rad) ./ (norm(rad)^3);
    grad = grad + cp;
    
    %% weighting by the parallel and perpendicular vectors
    grad_final(1) = alfa*grad(1);
    grad_final(2) = alfa*grad(2);
    grad_final(3) = beta*grad(3);
    
    %% normalization and speed control
    grad_final = gamma * grad_final ./ norm(grad_final);
    calculate_loop_properties;
    
    %% plotting
    [a_view, b_view] = view;
    plot3(p(1),p(2),p(3),'k.','MarkerSize', 30);
    view([a_view, b_view]); axis([-2 2,-2 2, -2 2]); 
    hold on, grid on
    p_traj(:,n) = p;
    plot3(p_traj(1,1:n), p_traj(2,1:n), p_traj(3,1:n), 'ro');
    quiver3(p(1),p(2),p(3),grad_final(1),grad_final(2),grad_final(3),'g');
    visualize_loop; 
    hold off
    
    %% move point with the gradient
    p = p + grad_final;
    
    %% add noise to loop xyz
    rx = 0 + 0.2.*randn(1, n_theta);
    ry = 0 + 0.2.*randn(1, n_theta);
    rz = 0 + 0.2.*randn(1, n_theta);
    x = xo + rx; y = yo + ry; z = zo + rz;
     
    %% add noise to loop rad z
%     rr = 0 + 0.2*randn(1, n_theta);
%     rz = 0 + 0.2*randn(1, n_theta);
%     x = xo + rr.*cos(theta); y = yo + rr.*sin(theta); z = zo + rz;
%     x = xo + (rr.^2).*cos(theta); y = yo + (rr.^2).*sin(theta); z = zo + rz;
    
    if abs(p(3)) < 0.05
        dist = sqrt(p(1)^2 + p(2)^2);
        if dist < radius
            disp('Successful Insertion')
            dist
            break
        else
            disp('Failed Insertion')
            exit(-1)
        end
    end
    pause(0.1);
    drawnow
    if output_video == 1
        frame = getframe;
        writeVideo(writerObj,frame);
    end
end
if output_video == 1
    close(writerObj);
end

close all





