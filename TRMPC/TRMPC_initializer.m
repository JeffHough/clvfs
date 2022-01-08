% Just messing around to get the TRMPC working...
% IF we assume that "W" can occur in arbitrary direction, problem reduces
% to finding eigenvalues of A, and then checking (lambda_max)^s < 0.01

% NOTE - THE STATES WILL BE MODELLED AS [POS_VECTOR; VEL_VECTOR]

%% Simulation meta-data:

freq_control = 10; %Hz
FixedStep = 1/freq_control;

%% Hill's dynamics:

MU = 3.986*10^5; % km^3/s^2
a = 7000; %km

n = sqrt(MU/a^3);

BL= [3*n^2      0       0;
     0          0       0;
     0          0    -n^2];
 
BR = [0         2*n     0;
      -2*n      0       0;
      0         0       0];
 
A_c = [zeros(3), eye(3);
      BL        BR    ];
  
B_c = [zeros(3);eye(3)];

C = eye(6);
D = zeros(6,3);

% Convert the continuous system into a discrete one:
sys = ss(A_c, B_c, C, D);
sys_d = c2d(sys, FixedStep, 'zoh');

A_d = sys_d.A;
B_d = sys_d.B;

%% Select an LQR controller

R = 10*eye(3);
Q = 10*eye(6);

% Pick the Q and R matrices - solve the P and K matrix:
[K, P, ~] = lqr(sys_d, Q, R);
K = -K;

%% Tightening constraints on the INPUTS for the Nominal system (MPC):

Au = [eye(3);-eye(3)];
u_max_scalar = 1; % [m/s^2]
u_max = ones(6,1)*u_max_scalar;

% Now, need to solve the linear program:
% FOR EACH ELEMENT OF b_u:
% minimize -(Au*K)(iRow,:) * e

premult_matrix = Au*K;
bu = zeros(6,1);

% CREATE MATRIX AND VECTOR TO DESCRIBE THE TRACKING ERROR SET:
% max TRACKING error vector:
e_max_vect = [0.1 ; 0.1 ; 0.1 ; 0.05 ; 0.05 ; 0.05];
error_constraint_vector = [e_max_vect;e_max_vect];

% max tracking error matrix
error_constraint_matrix = [eye(6);-eye(6)];

% solve for the reduced MPC constraint vector:
u_max_mpc = return_reduced_constraints(premult_matrix, error_constraint_matrix, error_constraint_vector, u_max);


%% Solving the mRPI state set:
% ----------------- For the dynamics system ------------------
alpha = 0.01; % The size of the disturbance that we will allow.

% Get the dynamics system with the control gain:
A_controlled = A_d + B_d*K;

% Get the maximum eigen value (and its norm)
eigs = eig(A_controlled);
eigs_norms = abs(eigs);
max_eig = max(eigs_norms);

% solve for "s" - the multiple until the eigen value < alpha:
s_control = ceil(log(alpha)/log(max_eig));

% --------------- For the observer system --------------------
L = 0.5*eye(6);
A_observer = A_d - L*C;
max_eig = max(abs(eig(A_observer)));

% solve for "s" of the observer.
s_observer = ceil(log(alpha)/log(max_eig));

%% Now, we can get the inequality matrix, solve the mRPI, and then get the inequality vector.

% An initial x_con_matrix - just for size.
x_con_mat = zeros(10, 6);

% The location of the docking port:
d = [10 ; 0 ; 0];

% The matrix and vector describing the cone:
[A_cone, b_cone] = return_square_cone(pi/6, d);

% the radius of the target:
rs = 10;

% the maximum speed:
v_max = [1;1;1];

% Choose some random rotation matrix for the cone (use this to get o_hat in
% CLVF)
C_CB = C3(pi/8)*C1(pi/7);






