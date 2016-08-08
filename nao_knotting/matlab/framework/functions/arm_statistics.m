function [ ] = arm_statistics( A, S, E, n, N )
%ARMSTATISTICS This function plots all the attributes associated with the arm movement
%   There are 6 default plots: 3 for the speed of each joint & 3 for the
%   angle of each joint, the other 3 plots in black are optional, they show
%   the position in each axis of the effector
t=1:N;                              %creates the time axis for the plot
subplot(1,2,2)                      %defines a subplot zone
hold on                             %keeps the plotted data on screen
axis([0 N -5 +5]);                  %set the range for the plot

plot(t(1:n),A(1,1:n),'b');          %angle a1
plot(t(1:n),S(1,1:n),'b:');         %speed of angle a1

plot(t(1:n),A(2,1:n),'r');          %angle a2
plot(t(1:n),S(2,1:n),'r:');         %speed of angle a2

plot(t(1:n),A(3,1:n),'g');          %angle a3
plot(t(1:n),S(3,1:n),'g:');         %speed of angle a3

%plot(t(1:n),E(1,1:n),'k-');        %position in x of effector
%plot(t(1:n),E(2,1:n),'k:');        %position in y of effector
%plot(t(1:n),E(3,1:n),'k-.');       %position in z of effector

legend('alpha1 (rad)','speed alpha1 (rad/s)','alpha2 (rad)','speed alpha2 (rad/s)','alpha3 (rad)','speed alpha3 (rad/s)');

end
