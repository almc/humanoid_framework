function [ u v ] = effcamera( p, f, angles, up, vp, w, h )
%EFFCAMERA This function draws what the camera sees given the location of the robotic arm
%   The only extra transformation is R4 which consist of a rotation to
%   match the axes of the camera with the axes of the given joint.
%   this function was not used however it can be adapted to simulate a camera.
a1=angles(1);a2=angles(2);a3=angles(3);a4=pi/2; %load angles
L1=1.3; L2=1.2; L3=1.1;                 %arms length
tx1=0;          ty1=0; tz1=L1;          %first translation
tx2=L2*sin(a2); ty2=0; tz2=L2*cos(a2);  %second translation
tx3=L3*sin(a3); ty3=0; tz3=L3*cos(a3);  %third translation
tx4=0;          ty4=0; tz4=0;           %fourth translation (if apply)

R1=[cos(a1) -sin(a1) 0 tx1;             %Z axis rotation
    sin(a1)  cos(a1) 0 ty1;             %& first translation
    0        0       1 tz1;
    0        0       0  1];

R2=[cos(a2) 0  sin(a2) tx2;             %Y axis rotation
    0       1    0     ty2;             %& second translation
   -sin(a2) 0  cos(a2) tz2;
    0       0    0      1];

R3=[cos(a3) 0  sin(a3) tx3;             %Y axis rotation
    0       1    0     ty3;             %& third translation
   -sin(a3) 0  cos(a3) tz3;
    0       0    0      1];

R4=[cos(a4) sin(a4) 0 tx4;             %Z axis camera rotation
    -sin(a4)  cos(a4) 0 ty4;             %& no translation
    0        0       1 tz4;
    0        0       0  1];

T=R1*R2*R3*R4;Rc=T\[p;1];               %creates transformation from World->Camera coord
xc=Rc(1);yc=Rc(2);zc=Rc(3);             %find and save the coord of p in Camera coord
uc=xc*f/zc;vc=yc*f/zc;                  %calculate uc & vc
u=uc+up;v=vc+vp;                        %calculate u & v

subplot(1,2,2)                          %reffers to subplot location
title('Camera view');                   %load title of plot
xlabel('u');ylabel('v');                %label all the axes.
hold on                                 %keeps the plotted data on screen
grid on                                 %put grid
axis([0 w 0 h]);                        %set the range for the plot
plot(u,v,'ko','MarkerFaceColor','m');   %plot the dot in the camera perspective
set(gca,'DataAspectRatio',[1 1 1]);     %keep default aspect ratio without stretching

end
