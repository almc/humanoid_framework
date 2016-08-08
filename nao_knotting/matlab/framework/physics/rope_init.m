%% rope variables 1
rope.L      = 2.5;%2.5;
rope.N      = 30;
rope.E      = rope.N - 1;
rope.dl     = rope.L / rope.E;
rope.q      = linspace(0, rope.L, rope.N);
rope.M      = 0.01;
rope.dm     = rope.M / rope.N * ones(1,rope.N);

% rope.pos    = [+1.0*rope.q - 4;
%                +0.0*rope.q + 0;
%                +0.0*rope.q + 1];

rope.pos    = [+1.0*rope.q - 3.5 + loop.center(1);     % -3.0
               +0.0*rope.q - 0   + loop.center(2);     % -0.5  [-1.5;1.0]
               +0.0*rope.q + 2.0];                     % +1.0  [ 0.0;4.0]
           
rope.vel    = [0*rope.q;
               0*rope.q;
               0*rope.q];

%% rope constants 1
g = [  0,  0, -0.01];
k_spring_compression =   0.002;
k_spring_friction    =  0.0001;
k_floor_friction     =     0.0;
% k_spring_compression =   0.05;
% k_spring_friction    =  0.005;
% k_floor_friction     =    0.0;


%% arm grasping constants 1
k_grasp = 30;
k_hold  = 0;

%% collision variables 1
coll_loop_rope = zeros(loop.N, rope.N);
coll_rope_rope = zeros(rope.N, rope.N);

n_points_discarded = 5; %15 %5
k_repulsion_loop = 0.015;
k_repulsion_rope_long_comp = 0.0;
k_repulsion_rope_long_exte = 0;
k_repulsion_rope_trav_cros = 0.040;

thr_loop_rope = 0.2; % 6.0 * rope.dl; % depends on the discretization of the loop
thr_rope_rope_long_comp =  0.0 * rope.dl; % not less than 0.5, not more than 1.0
thr_rope_rope_long_exte =  0.0 * rope.dl; % stretch limit of the rope 1.5*dl
thr_rope_rope_trav_cros = 0.1; %0.3; %0.1; % 3.0 * rope.dl; % distance to prevent self crossing % 2.5 and 10 units
thr_rope_loop_strangled = 0.6; %1.4; %0.60; % 4.0 * rope.dl; % distance to say the rope loop is fixed

n_arms = 2;
grasp_id_rope = zeros(n_arms, rope.N);
% grasp_id_rope(2,1)   = 1;
% grasp_id_rope(1,end) = 1;
rope_loop_idx = [];
loop_strangled = 0;


% %% rope variables 2
% rope.L      = 2.5;
% rope.N      = 70;
% rope.E      = rope.N - 1;
% rope.dl     = rope.L / rope.E;
% rope.q      = linspace(0, rope.L, rope.N);
% rope.M      = 1;
% rope.dm     = rope.M / rope.N * ones(1,rope.N);
% 
% rope.pos    = [1.0*rope.q - 4;
%                0.0*rope.q + 0;
%                0.0*rope.q + 0];
% 
% rope.vel    = [0*rope.q;
%                0*rope.q;
%                0*rope.q];           
%            
% %% rope constants 2
% g = [  0,  0, -1];
% k_spring_compression =   35;
% k_spring_friction    =  0.2;
% k_floor_friction     = -2.0;
% rope_dt              = 0.01;
% 
% %% arm grasping constant 2
% k_grasp              =  100;
% k_hold               =    1;
% 
% %% collision variables 2
% coll_loop_rope = zeros(loop.N, rope.N);
% coll_rope_rope = zeros(rope.N, rope.N);
% 
% k_repulsion_loop = 10;
% k_repulsion_rope_long_comp = 10;
% k_repulsion_rope_long_exte = -1;
% k_repulsion_rope_trav_cros = 25;
% 
% thr_loop_rope = 12.0 * rope.dl; % depends on the discretization of the loop
% thr_rope_rope_long_comp = 0.7 * rope.dl; % not less than 0.5, not more than 1.0
% thr_rope_rope_long_exte = 1.0 * rope.dl; % stretch limit of the rope 1.5*dl
% thr_rope_rope_trav_cros = 4.5 * rope.dl; % distance to prevent self crossing % 2.5 and 10 units
% thr_rope_loop_strangled = 14.0 * rope.dl; % distance to say the rope loop is fixed
% 
% n_arms = 2;
% grasp_id_rope = zeros(n_arms, rope.N);
% % grasp_id_rope(1,1)   = 1;
% % grasp_id_rope(2,end) = 1;
% rope_loop = [];
% loop_strangled = 0;