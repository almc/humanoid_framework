function [ traj ] = draw_trajectories(n, arm1, arm2, traj)

%% store joint position, angles and speed for each timestep - arm1
    traj.arm1.EF(:,n) = arm1.ef;      traj.arm1.J1(:,n) = arm1.j1;
    traj.arm1.J2(:,n) = arm1.j2;      traj.arm1.J3(:,n) = arm1.j3;
    traj.arm1.SP(:,n) = arm1.alpha_v; traj.arm1.AL(:,n) = arm1.alpha;
    plot3(traj.arm1.EF(1,1:n), traj.arm1.EF(2,1:n), traj.arm1.EF(3,1:n), 'r:');
%     plot3(traj.arm1.J3(1,1:n), traj.arm1.J3(2,1:n), traj.arm1.J3(3,1:n), 'k');

%% store joint position, angles and speed for each timestep - arm2
    traj.arm2.EF(:,n) = arm2.ef;      traj.arm2.J1(:,n) = arm2.j1;
    traj.arm2.J2(:,n) = arm2.j2;      traj.arm2.J3(:,n) = arm2.j3;
    traj.arm2.SP(:,n) = arm2.alpha_v; traj.arm2.AL(:,n) = arm2.alpha;
    plot3(traj.arm2.EF(1,1:n), traj.arm2.EF(2,1:n), traj.arm2.EF(3,1:n), 'b:');
%     plot3(traj.arm2.J3(1,1:n), traj.arm2.J3(2,1:n), traj.arm2.J3(3,1:n), 'k');

end

