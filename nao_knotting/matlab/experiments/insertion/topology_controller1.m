vz = 10; % velocity constant
a = -32; % acceleration constant

r = 1;
t = 0:.1:2*pi;
%z = vz*t + 1/2*a*t.^2;

x = r*cos(-t);
y = r*sin(-t);
z = r/2*cos(pi*t);

u = gradient(x);
v = gradient(y);
w = gradient(z);
scale = 0;

figure
quiver3(x,y,z,u,v,w,scale)
hold on
set(gcf, 'PaperSize', [10 14]);

p  = [2; 2; 0.1];

iterations = 50;

               
% mov = avifile('topology_controller1.avi'); 
                       
for iter = 1:iterations
    grad = [0; 0; 0];
    for i = 1:length(u)
        pl = [x(i); y(i); z(i)];
        dl = [u(i); v(i); w(i)];
        r  = p - pl;
        cp = cross(dl,r) ./ (norm(r)^3);
        grad = grad + cp;
        quiver3(p(1),p(2),p(3),cp(1),cp(2),cp(3),scale,'r');
    end
    grad = grad ./ length(u);
    if norm(grad) > 0.1
        grad = 0.1 * grad ./ norm(grad);
    end
    if norm(grad) < 0.05
        grad = 0.1 * grad ./ norm(grad);
    end
    quiver3(p(1),p(2),p(3),grad(1),grad(2),grad(3),scale,'g');
    p = p + grad;
    plot3(p(1),p(2),p(3),'ko');
    drawnow
%     frame = getframe(gcf);
%     mov = addframe(mov,frame); 
end
% mov = close(mov);