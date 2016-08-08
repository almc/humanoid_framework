clear all, close all
r = 1;
% t = 0:.1:2*pi;
t = 0:.1:1.9*pi;
%z = vz*t + 1/2*a*t.^2;
l = -1:0.1:1;
x = r*cos(-t);
y = r*sin(-t);
z = r/2*cos(pi*t);

% x = l;
% y = l;
% z = 0*l;

gx = gradient(x);
gy = gradient(y);
gz = gradient(z);

perp_vector = [0,0,0]';
loop_center = [sum(x)./length(x);
               sum(y)./length(y);
               sum(z)./length(z)];

vector_length = 1;

for i = 1:length(gx)-1
    perp_vec_local = cross( [gx(i), gy(i), gz(i)]', [gx(i+1), gy(i+1), gz(i+1)]' );
    perp_vector = perp_vector + perp_vec_local;
end
parl_vector1 = [x(1) - loop_center(1);
                y(1) - loop_center(2);
                z(1) - loop_center(3)];
parl_vector2 = cross(perp_vector, parl_vector1);

perp_vector  = vector_length*-perp_vector  ./ norm(perp_vector,2);
parl_vector1 = vector_length*+parl_vector1 ./ norm(parl_vector1,2);            
parl_vector2 = vector_length*-parl_vector2 ./ norm(parl_vector2,2);


figure
view([1 1 1]);
axis([-1 1,-1 1, -1 1]);
axis([-2 2,-2 2, -2 2]);
% quiver3(x,y,z,u,v,w,scale)
hold on, grid on
plot3(x,y,z,'LineWidth', 4);
plot3([x(end),x(1)], [y(end),y(1)], [z(end),z(1)], 'r-.');

% plot3(loop_center(1), loop_center(2), loop_center(3), 'ko','MarkerFaceColor','r');

quiver3(loop_center(1), loop_center(2), loop_center(3), ...
        perp_vector(1), perp_vector(2), perp_vector(3), 'r');

quiver3(loop_center(1) , loop_center(2) , loop_center(3), ...
        parl_vector1(1), parl_vector1(2), parl_vector1(3), 'g');

quiver3(loop_center(1) , loop_center(2) , loop_center(3), ...
        parl_vector2(1), parl_vector2(2), parl_vector2(3), 'b');


% plot3([x(end), x(1)], [y(end), y(1)], [z(end), z(1)],'r', 'LineWidth', 4);

% set(gcf, 'PaperSize', [10 14]);


%% precalculation of the magnetic field over the grid
grid_step = 0.4;
[X_grid,Y_grid,Z_grid] = meshgrid(-2:grid_step:2, -2:grid_step:2, -2:grid_step:2);
U_grid = zeros(size(X_grid)); V_grid = zeros(size(Y_grid)); W_grid = zeros(size(Z_grid));


for i = 1:length(x)-1
    pl = [x(i); y(i); z(i)];
    dl = [x(i+1) - x(i); y(i+1) - y(i); z(i+1) - z(i)];
    for i1 = 1:size(X_grid,1)
        for i2 = 1:size(Y_grid,2)
            for i3 = 1:size(Z_grid,3)
                p = [X_grid(i1,i2,i3); Y_grid(i1,i2,i3); Z_grid(i1,i2,i3)];
                r  = p - pl;
                if r < 0.25
                    continue
                end
                cp = cross(dl,r) ./ (norm(r)^3);
%                 grad = grad + cp;
                U_grid(i1,i2,i3) = U_grid(i1,i2,i3) + cp(1);
                V_grid(i1,i2,i3) = V_grid(i1,i2,i3) + cp(2);
                W_grid(i1,i2,i3) = W_grid(i1,i2,i3) + cp(3);
            end
        end
    end
end
% mean_U_grid = abs(mean(mean(mean(U_grid))));
% mean_V_grid = abs(mean(mean(mean(V_grid))));
% mean_W_grid = abs(mean(mean(mean(W_grid))));
% 
% U_grid(U_grid > 1.5*mean_U_grid) = 0; U_grid(U_grid < -1.5*mean_U_grid) = 0;
% V_grid(V_grid > 1.5*mean_V_grid) = 0; V_grid(V_grid < -1.5*mean_V_grid) = 0;
% W_grid(W_grid > 1.5*mean_W_grid) = 0; W_grid(W_grid < -1.5*mean_W_grid) = 0;

quiver3(X_grid, Y_grid, Z_grid, U_grid, V_grid, W_grid, 1, 'k');

p  = [2; 2; 0.1];

iterations = 400;
alfa = 1.5;
beta = 0;
% alfa = 1.0;
% beta = 1.0;

% mov = avifile('topology_controller1.avi'); 

% pause(5);
for iter = 1:iterations
%     if iter == 180
%         beta = 0.1;
%         alfa = 0;
%     end
%     if iter == 200
%         beta = 0;
%         alfa = 1.5;
%     end
    grad = [0; 0; 0];
    for i = 1:length(x)-1
%         i
        pl = [x(i); y(i); z(i)];
        dl = [x(i+1) - x(i); y(i+1) - y(i); z(i+1) - z(i)];
        r  = p - pl;
        cp = cross(dl,r) ./ (norm(r)^3);
        grad = grad + cp;
    end
    
    norm(grad,2)
    %% virtual loop closing
    pl = [x(end); y(end); z(end)];
    dl = [x(1) - x(end); y(1) - y(end); z(1) - z(end)];
    r  = p - pl;
    cp = cross(dl,r) ./ (norm(r)^3);
    grad = grad + cp;
    
    grad = grad ./ length(x);
    
    %% normalization and visualization
    if norm(grad) > 0.1
        grad = 0.1 * grad ./ norm(grad);
    end
    if norm(grad) < 0.05
        grad = 0.1 * grad ./ norm(grad);
    end
    
    plot3(p(1),p(2),p(3),'ko','MarkerSize', 4);
    quiver3(p(1),p(2),p(3),grad(1),grad(2),grad(3),'g');

    p = p + grad;
%     p(1) = p(1) + alfa*grad(1);
%     p(2) = p(2) + alfa*grad(2);
%     p(3) = p(3) + beta*grad(3);

%     p(1) = p(1) + alfa*dot(grad, parl_vector1);
%     p(2) = p(2) + alfa*dot(grad, parl_vector2);
%     p(3) = p(3) + beta*dot(grad, perp_vector);

%     grad_resp_loop = [alfa * dot(grad, parl_vector1);
%                       alfa * dot(grad, parl_vector2);
%                       beta * dot(grad, perp_vector )];
%     
%     p(1) = p(1) + dot(grad_resp_loop,[1,0,0]');
%     p(2) = p(2) + dot(grad_resp_loop,[0,1,0]');
%     p(3) = p(3) + dot(grad_resp_loop,[0,0,1]');
    
    drawnow
%     frame = getframe(gcf);
%     mov = addframe(mov,frame); 
end
% mov = close(mov);