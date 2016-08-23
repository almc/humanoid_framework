%% youbot simulation
%  Author: Alejandro Marzinotto

%% initialization
clear all
close all
clc

%% time variables
T    = 120;
dt   = 0.1;
N    = round(T / dt) + 1;
time = linspace(0, T, N);

%% loop variables
biot_savart_init

%% rope variables
rope_init

%% youbot variables
% arm1
arm1.alpha   = [-90, -45, 60, 80, 0]' *pi/180;
arm1.alpha_v = [0,  0,  0,  0, 0]' *pi/180;
arm1.grad    = [0,  0,  0]';
arm1.ef = 0; arm1.j1 = 0; arm1.j2 = 0; arm1.j3 = 0; arm1.j4 = 0; arm1.j5 = 0;
arm1.alpha_l1 = [-pi, -pi/1.1, -pi/1.1, -pi/1.1, -pi/1.1]';
arm1.alpha_l2 = [ pi,  pi/1.1,  pi/1.1,  pi/1.1,  pi/1.1]';
% arm2
arm2.alpha   = [-90, -45, 60, 80, 0]' *pi/180;
arm2.alpha_v = [0,  0,  0,  0, 0]' *pi/180;
arm2.grad    = [0,  0,  0]';
arm2.ef = 0; arm2.j1 = 0; arm2.j2 = 0; arm2.j3 = 0; arm2.j4 = 0; arm2.j5 = 0;
arm2.alpha_l1 = [-pi, -pi/1.1, -pi/1.1, -pi/1.1, -pi/1.1]';
arm2.alpha_l2 = [ pi,  pi/1.1,  pi/1.1,  pi/1.1,  pi/1.1]';
% base
base.pos_rot = [2, 3, pi/4];
base.omega   = [0, 0, 0, 0]'; % w1:uplft, w2:uprght, w3:dwnrght, w4:dwnlft

%% youbot constants (length in decimeters)
arm1.l1 = 0.41; arm1.l2 = 1.55; arm1.l3 = 1.35; arm1.l4 = 1.13; arm1.l5 = 0.57; arm1.d = 1.43;
arm2.l1 = 0.41; arm2.l2 = 1.55; arm2.l3 = 1.35; arm2.l4 = 1.13; arm2.l5 = 0.57; arm2.d = 1.43;
% arm2.l1 = 1.3; arm2.l2 = 1.2; arm2.l3 = 1.1; arm2.d = 1.0;
base.w  = 1.0; base.l  = 2.0; base.r  = 0.5;
% base.w  = 3.16; base.l  = 4.56; base.r  = 0.79;


%% jacobian calculation per arm
[jacobian_arm1, J1_arm1, J2_arm1, J3_arm1, J4_arm1, J5_arm1, EF_arm1] = arm_jacobian(arm1, 1);
[jacobian_arm2, J1_arm2, J2_arm2, J3_arm2, J4_arm2, J5_arm2, EF_arm2] = arm_jacobian(arm2, 2);

%% references (used to move the base and arms)
arm1.ref   = [ +1.5 + 0*0.7*cos(linspace( 0, 2*pi, N)) ;
                1.0 + 0*        linspace( 0, 2*pi, N)  ;
                2.5 + 0*0.7*sin(linspace( 0, 2*pi, N))];

arm2.ref   = [ -1.5 + 0*0.7*cos(linspace(pi, 3*pi, N)) ;
                1.0 + 0*        linspace(pi, 3*pi, N)  ;
                2.5 + 0*0.7*sin(linspace(pi, 3*pi, N))];

base.ref   = [0.0 + 0*linspace(0,    2*pi, N);
              2.0 + 0*linspace(0,    2*pi, N);
              0   + 0*linspace(pi/2, 2*pi, N) ];

% base.ref   = [0.0 + 0*linspace(0, 2*pi, N);
%               2.0 + 0*linspace(0, 2*pi, N);
%               0*linspace(0,2*pi,N)       ];

arm1.range = arm_range(arm1.ref, base.ref, arm1);
arm2.range = arm_range(arm2.ref, base.ref, arm2);

%% controller parameters
Kp_arm    = 2.5;
Kp_base.x = 0.4; Kp_base.y = 0.4; Kp_base.rot = 1.4;

%% hybrid controller variables
state = 0;

%% trajectory variables
traj.arm1.EF = zeros(3,N); traj.arm1.J1 = zeros(3,N);
traj.arm1.J2 = zeros(3,N); traj.arm1.J3 = zeros(3,N);
traj.arm1.SP = zeros(3,N); traj.arm1.AL = zeros(3,N);

traj.arm2.EF = zeros(3,N); traj.arm2.J1 = zeros(3,N);
traj.arm2.J2 = zeros(3,N); traj.arm2.J3 = zeros(3,N);
traj.arm2.SP = zeros(3,N); traj.arm2.AL = zeros(3,N);

%% screen configuration
% scrsz = get(0,'ScreenSize');
% figure('OuterPosition',[scrsz(3)/2 scrsz(4)/2 scrsz(3)/2 scrsz(4)/2])
% view([4 2 2]);
% view([2 1 1]);

view([130,42]);
% view([123, 70]);

output_video = 0;
if output_video == 1
    filename = 'experiment3.mp4';
    profile  = 'MPEG-4'; %'Uncompressed AVI';
    % writerObj = VideoWriter(filename, profile);
    writerObj = VideoWriter(filename);%, profile);
    writerObj.FrameRate = 24;
    % writerObj.Quality   = 50;
    open(writerObj);
end

calc_reachability = 0;
done_twist1 = 0;

%% debug
debug.point1 = zeros(3,40);
debug.grad1  = [0;0;0];

debug.point2 = zeros(3,40);
debug.grad2  = [0;0;0];

debug.point3 = zeros(3,40);
debug.grad3  = [0;0;0];

%% main loop
% profile on
for n = 1:N
    t = time(n);
    n

    %% calculate joint position given joint angles
    [arm1.j1, arm1.j2, arm1.j3, arm1.j4, arm1.j5, arm1.ef] = ...
        fk_youbot(arm1.alpha, base.pos_rot, J1_arm1, J2_arm1, J3_arm1, J4_arm1, J5_arm1, EF_arm1);
	[arm2.j1, arm2.j2, arm2.j3, arm2.j4, arm2.j5, arm2.ef] = ...
        fk_youbot(arm2.alpha, base.pos_rot, J1_arm2, J2_arm2, J3_arm2, J4_arm2, J5_arm2, EF_arm2);

    %% test arm reachability
    if calc_reachability == 1
        test_arm_reachability()
    end

    %% calculate arm reference using biot_savart law, note: this function is wrong,
    % check biot_savart_run2 and 3. These are now called inside hybrid_controllerx
    % [arm1.grad, arm1.ref(:,n)] = biot_savart_run1(arm1.ef, loop.pos,     loop, biot_savart);
    % [arm2.grad, arm2.ref(:,n)] = biot_savart_run1(arm2.ef, loop.pos_rev, loop, biot_savart);

    %% simulate rope timestep
    rope_run

    %% simulate anchoring entity dynamic
    anchoring_entity_run

    %% hybrid control for the arms and base together
    % usage: hybrid_controller#
    hybrid_controller4

    %% draw everything
    [a_view, b_view] = view;
    plot3(0, 0, 0,'g');
%     view([a_view, b_view]); axis([-6 +6, -1 +6, +0 +6]);
    view([a_view, b_view]); axis([-3 +3, -1 +5, +0 +6]);
%     view([a_view, b_view]); axis([-1 +7, -1 +5, +0 +4]);
%     view([a_view, b_view]); axis([-7 +7, -7 +7, +0 +7]);
    %axis([-2 +2, -2 +2, +0 +2]);
    hold on, grid on
    %% draw the robot
    draw_youbot(base, arm1, arm2);
%     [traj] = draw_trajectories(n, arm1, arm2, traj);

    %% draw youbot arm references
    plot3(arm1.ref(1,n), arm1.ref(2,n), arm1.ref(3,n), 'ro');
    plot3(arm2.ref(1,n), arm2.ref(2,n), arm2.ref(3,n), 'bo');
    plot3(base.ref(1,n), base.ref(2,n), 0.01,          'co');
    quiver3(arm1.ef(1), arm1.ef(2), arm1.ef(3), arm1.ref(1,n)-arm1.ef(1), arm1.ref(2,n)-arm1.ef(2), arm1.ref(3,n)-arm1.ef(3), 'r-.');
    quiver3(arm2.ef(1), arm2.ef(2), arm2.ef(3), arm2.ref(1,n)-arm2.ef(1), arm2.ref(2,n)-arm2.ef(2), arm2.ref(3,n)-arm2.ef(3), 'b-.');

    quiver(base.ref(1,n)  , base.ref(2,n)  , 2*cos(base.ref(3,n))  , 2*sin(base.ref(3,n))  ,'b');
    quiver(base.pos_rot(1), base.pos_rot(2), 2*cos(base.pos_rot(3)), 2*sin(base.pos_rot(3)),'r');

    %% draw biot savart
    biot_savart_plot
    rope_plot
    hold off

    %% continuous control for the arms and base separately
    [arm1.alpha_v] = arm_controller (arm1.ef, arm1.ref(:,n), arm1.alpha, ...
                                     base.pos_rot, Kp_arm, jacobian_arm1, grasp_id_rope(1,:));
    [arm2.alpha_v] = arm_controller (arm2.ef, arm2.ref(:,n), arm2.alpha, ...
                                     base.pos_rot, Kp_arm, jacobian_arm2, grasp_id_rope(2,:));
    [base.omega]   = base_controller(base.pos_rot, base.ref(:,n), Kp_base, base);

    %% update youbot state after dt given alpha_v and omega
    [base.pos_rot, arm1.alpha, arm2.alpha] = youbot_dynamics(base, arm1, arm2, dt);

    %% wait draw and clear figure
    pause(0.0005);
    drawnow
    if output_video == 1
        frame = getframe;
        writeVideo(writerObj,frame);
    end
end
if output_video == 1
    close(writerObj);
end

% profile viewer
close all
