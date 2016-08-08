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

std_dev_errors = [0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30];
R = size(std_dev_errors,2);  % number of noise scenarios
M = 1%1000; % number of experiments
N = 200; % number of iterations per experiment

p_traj = zeros(3,N);
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

d_to_center = zeros(R,M);
i_to_insert = zeros(R,M);
f_to_insert = zeros(R,M);

for r = 1:R
    r
    stdev = std_dev_errors(r)
    for m = 1:M
        m
        p  = p_init;
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
            p_traj(:,n) = p;
            plot3(p_traj(1,1:n), p_traj(2,1:n), p_traj(3,1:n), 'ro');
            quiver3(p(1),p(2),p(3),grad_final(1),grad_final(2),grad_final(3),'g');
            visualize_loop;
            hold off

            %% move point with the gradient
            p = p + grad_final;

            %% section where the noise is added isotropically or cylindrically
            %% add noise to loop xyz
%             rx = 0 + stdev.*randn(1, n_theta);
%             ry = 0 + stdev.*randn(1, n_theta);
%             rz = 0 + stdev.*randn(1, n_theta);
%             x = xo + rx; y = yo + ry; z = zo + rz;

            %% add noise to loop rad z
            rr = 0 + stdev*randn(1, n_theta);
            rz = 0 + stdev*randn(1, n_theta);
            x = xo + rr.*cos(-theta); y = yo + rr.*sin(-theta); z = zo + 0*rz;

            if abs(p(3)) < 0.05
                dist = sqrt(p(1)^2 + p(2)^2);
                if dist < radius
                    disp('Successful Insertion')
                    d_to_center(r,m) = dist;
                    i_to_insert(r,m) = n;
                    break
                else
                    disp('Failed Insertion')
                    f_to_insert(r,m) = 1;
                    break
                end
            end
%             pause(0.1);
            drawnow
            if output_video == 1
                frame = getframe;
                writeVideo(writerObj,frame);
            end
        end
    end
end



if output_video == 1
    close(writerObj);
end

plot_results = 0

if plot_results == 1
    figure
    subplot(1,2,1)
    yd = mean(d_to_center,2);
    ed = std(d_to_center,1,2);
    h = errorbar(std_dev_errors,yd,ed, 'Marker', 'o')
    hc = get(h, 'Children')
    set(hc(1),'color','b', 'LineWidth', 2) % data
    set(hc(2),'color','r') % error bars
    axis([0 0.35,0 1]);
    xlabel('noise standard deviation \sigma')
    ylabel('distance to center (m)')
    title('Insertion Quality')
    grid on
    subplot(1,2,2)
    yi = mean(i_to_insert,2);
    ei = std(i_to_insert,1,2);
    h = errorbar(std_dev_errors,yi,ei, 'Marker', 'o')
    hc = get(h, 'Children')
    set(hc(1),'color','b', 'LineWidth', 2) % data
    set(hc(2),'color','r') % error bars
    axis([0 0.35,0 100]);
    xlabel('noise standard deviation \sigma')
    ylabel('number of iterations until insertion')
    title('Insertion Delay')
    grid on
end

k = waitforbuttonpress

close all
