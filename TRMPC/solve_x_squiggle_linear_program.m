function x_squiggle = solve_x_squiggle_linear_program(G_i, s1, alpha1, A_L, L, A_w, b_w, A_v, b_v)
% Just going to use as a sub-function for the larger mRPI program.
% See the description of all the variables there.

%% Start

% set optimizer options:
options = optimoptions('linprog','Algorithm','interior-point', 'Display','off');

% Set the s-vector size to the same as G_i:
x_squiggle = zeros(size(G_i));

% Initialize the A_L^0 power:
A_L_power = eye(numel(x_squiggle));

% Loop through 0 to s1-1, solving the linear programs:
for iPower = 0:s1-1
    % vector for w:
    w_constraint_vector = A_L_power'*G_i;
    
    % vector for v:
    v_constraint_vector = -L'*w_constraint_vector;
    
    % solve the linear programs:
    w = linprog(-w_constraint_vector, A_w, b_w, [], [], [], [], options);
    v = linprog(-v_constraint_vector, A_v, b_v, [], [], [], [], options);
    
    % add these to the x_squiggle vector:
    x_squiggle = x_squiggle + w + v;
    
    % Up the power of A_L_power:
    A_L_power = A_L_power*A_L;
    
end

% divide by the alpha:
x_squiggle = (1/(1-alpha1))  *  x_squiggle;


end