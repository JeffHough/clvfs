function MPCStructure = getMPCStructure(ICStructure, SpacecraftStructure, R, Q, Q_final, u_max_scalar, v_max)

A_d = ICStructure.A_d;
B_d = ICStructure.B_d;
b_cone = SpacecraftStructure.b_cone;

% CHOOSE THE TIME HORIZON:
Np = 10;

% GET THE EQUALITY MATRIX:
MPCStructure.Aeq = return_equality_mat(A_d, B_d, Np);

%% Build the cost matrix (based on LQR gains):
MPCStructure.Q = return_cost_matrix(R, Q, Np, Q_final);

%% Describing the INPUT constraints.
% NOTE - this will now be incorperated as a LB-UB combo.
MPCStructure.u_con_mat = [];
MPCStructure.u_max_scalar = u_max_scalar; % [m/s^2]
MPCStructure.u_con_vec = [];

%% Set up the state constraints:
% Only a 4x6 for this one:
MPCStructure.x_con_mat = zeros(4, 6);

% The actual state-constraint vector:
MPCStructure.x_con_vec = b_cone;

%% Create the LB and UB vectors:
% input and state lower bounds and upper bounds:
u_ub = [u_max_scalar;u_max_scalar;u_max_scalar];
u_lb = -u_ub;

x_ub = [Inf; Inf; Inf; v_max ; v_max ; v_max];
x_lb = -x_ub;

lb_vector = [u_lb ; x_lb];
ub_vector = [u_ub ; x_ub];

[MPCStructure.LB, MPCStructure.UB] = return_lb_and_ub_vectors(lb_vector, ub_vector, Np);

%% Some final dimensionality things:

% Size of total vector per step:
dimPerStep = numel(u_ub) + numel(x_ub);

% The total dimension of the X-vector:
MPCStructure.X_SIZE = (dimPerStep) * Np;

% Total number of equality contraints:
MPCStructure.N_EQ = Np*size(A_d, 1);

% Number of inequality constraints on the inner MPC:
MPCStructure.N_INEQ_INNER = Np * (size(MPCStructure.u_con_mat, 1) + size(MPCStructure.x_con_mat, 1));

% Get the number of constraints in the outer (Just the plane constraint @ each step).
MPCStructure.N_INEQ_OUTER = Np;

% Initial guess for the MPC
MPCStructure.X0 = zeros(MPCStructure.X_SIZE,1);

% Build the warm-start matrix:
MPCStructure.warm_start_matrix = return_warm_start_matrix(dimPerStep, Np);

end