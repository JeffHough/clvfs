% Just messing around to get the TRMPC working...
% IF we assume that "W" can occur in arbitrary direction, problem reduces
% to finding eigenvalues of A, and then checking (lambda_max)^s < 0.01

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


%% Solving the linear programs for the input system:

Au = [eye(3);-eye(3)];
u_max_scalar = 1; % [m/s^2]
u_max = ones(6,1)*u_max_scalar;

R = 10*eye(3);
Q = 10*eye(6);

% Pick the Q and R matrices - solve the P and K matrix:
[K, P, ~] = lqr(sys_d, Q, R);

% Now, need to solve the linear program:
% FOR EACH ELEMENT OF b_u:
% minimize -(Au*K)(iRow,:) * e

premult_matrix = Au*K;
bu = zeros(6,1);

% max error vector:
e_max_vect = [0.5 ; 1.0 ; 0.7 ; 0.1 ; 0.05 ; 0.2];
constraint_vector = [e_max_vect;e_max_vect];

constraint_matrix = [eye(6);-eye(6)];

options = optimoptions('linprog','Algorithm','interior-point', 'Display','off');

for iRow = 1:numel(bu)
   % Get the f-vector:
   f = -premult_matrix(iRow, :)';
   
   % Get the constraint matrix...
   e_worst_case = linprog(f, constraint_matrix, constraint_vector, [], [], [], [], options);
   
   % compute the worst-case, and fill in bu:
   bu(iRow) = -f'*e_worst_case;
    
end


%% Solving the mRPI sets:
max_disturbance = 1; % m/s^2
alpha = 0.01; % The size of the disturbance that we will allow.

% get the maximum eigenvalue:















