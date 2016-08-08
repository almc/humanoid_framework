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

alfas1 = [1.0, 1.414, 2.0, 10.0, 100.0];
%alfas2 = [1.0, 0.707, 0.5,  0.1,  0.01];
betas  = [1.0,   1.0, 1.0,  1.0,   1.0];
M = size(betas,2);
% M = 10; % number of experiments
N = 200; % number of iterations per experiment

p_traj = zeros(3,N,M); % for each alfa/beta combo, the trajectory
p_traj_plot = zeros(3,N);

p_init = [2; 0; 2];
% alfa = 1.5;
% beta = 0;
alfa = 1.0;
beta = 1.0;
gamma = 0.05;
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

% d_to_center = zeros(R,M);
i_to_insert = zeros(M);

stdev = 0.0;
for m = 1:M
    m
    p  = p_init;
    alfa = alfas1(m);
    beta = betas(m);
    for n = 1:N
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
        p_traj(:,n,m) = p;
%         p_traj_plot(:,n) = p;
%         plot3(p_traj_plot(1,1:n), p_traj_plot(2,1:n), p_traj_plot(3,1:n), 'ro');
        plot3(p_traj(1,1:n,m), p_traj(2,1:n,m), p_traj(3,1:n,m), 'ro');
        quiver3(p(1),p(2),p(3),grad_final(1),grad_final(2),grad_final(3),'g');
        visualize_loop;
        hold off

        %% move point with the gradient
        p = p + grad_final;

        %% add noise to loop xyz
        rx = 0 + stdev.*randn(1, n_theta);
        ry = 0 + stdev.*randn(1, n_theta);
        rz = 0 + stdev.*randn(1, n_theta);
        x = xo + rx; y = yo + ry; z = zo + rz;

        %% add noise to loop rad z
    %     rr = 0 + stdev*randn(1, n_theta);
    %     rz = 0 + stdev*randn(1, n_theta);
    %     x = xo + rr.*cos(theta); y = yo + rr.*sin(theta); z = zo + rz;
    %     x = xo + (rr.^2).*cos(theta); y = yo + (rr.^2).*sin(theta); z = zo + rz;

        if abs(p(3)) < 0.05
            dist = sqrt(p(1)^2 + p(2)^2);
            if dist < radius
                disp('Successful Insertion')
                i_to_insert(m) = n;
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
end


if output_video == 1
    close(writerObj);
end


plot_results = 0

if plot_results == 1
    figure
    hold on, grid on
    view([1 1 1]);
    axis([-2 2,-2 2, -2 2]);
    visualize_loop;
    plot3(p_traj(1,1:i_to_insert(1),1), p_traj(2,1:i_to_insert(1),1), p_traj(3,1:i_to_insert(1),1), 'r', 'LineWidth', 2.0);
    plot3(p_traj(1,1:i_to_insert(2),2), p_traj(2,1:i_to_insert(2),2), p_traj(3,1:i_to_insert(2),2), 'g', 'LineWidth', 2.0);
    plot3(p_traj(1,1:i_to_insert(3),3), p_traj(2,1:i_to_insert(3),3), p_traj(3,1:i_to_insert(3),3), 'b', 'LineWidth', 2.0);
    plot3(p_traj(1,1:i_to_insert(4),4), p_traj(2,1:i_to_insert(4),4), p_traj(3,1:i_to_insert(4),4), 'c', 'LineWidth', 2.0);
    plot3(p_traj(1,1:i_to_insert(5),5), p_traj(2,1:i_to_insert(5),5), p_traj(3,1:i_to_insert(5),5), 'y', 'LineWidth', 2.0);
    xlabel('x (meters)')
    ylabel('y (meters)')
    zlabel('z (meters)')
    % title('Insertion Trajectory for \alpha >= \beta')
    title('Insertion Trajectory for \alpha <= \beta')
  end

k = waitforbuttonpress

close all
