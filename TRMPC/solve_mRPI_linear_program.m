function [s] = solve_mRPI_linear_program(G_i, s1, s2, alpha1, alpha2, A_L, C, L, A_K, A_w, b_w, A_v, b_v)
%SOLVE_MRPI_LINEAR_PROGRAM will take in a vector G_i, and return the vector
%"s" which maximizes the dot-product with G_i.
%
% INPUTS:
%   G_i - the vector we dot product with.
%   s1 - solves solution A_L^s1 < alpha_1.
%   s2 - similar, but for A_K^s2 < alpha_2.
%   alpha1 - an accuracy described above.
%   alpha2 " ".
%   A_L - the dynamics matrix of the estimation error.
%   C - the observation matrix.
%   L - the observer gain.
%   A_K - the dynamics matrix of the tracking error.
%   A_w - the constraints matrix of the dynamics disturbance.
%   b_w - the constraints vector of the dynamics disturbance.
%   A_v - the constraints matrix of the measurement disturbance.
%   b_v - the constraints vector of the dynamics disturbance.
%
% OUTPUTS:
%   s - the vector within disturbance bounds with the LARGEST dot-product
%   with G_i.

%% START:

% set the optimization options:
options = optimoptions('linprog','Algorithm','interior-point', 'Display','off');

% set the s-vector size to the same as G_i:
s = zeros(size(G_i));

% Initialize the A_K^0 power:
A_K_power = eye(numel(s));

% Loop through 0 to s2-1, solving the linear programs:
for iPower = 0:s2-1
   % vector for x_squiggle:
   x_squiggle_constraint_vector = C'*L'*A_K_power'*G_i;
    
   % vector for v:
   v_constraint_vector = L'*A_K_power'*G_i;
   
   % solve these linear programs:
   x_squiggle = solve_x_squiggle_linear_program(-x_squiggle_constraint_vector, s1, alpha1, A_L, L, A_w, b_w, A_v, b_v);
   v = linprog(-v_constraint_vector, A_v, b_v, [], [], [], [], options);
   
   s = s + x_squiggle + v;
   
   % Up the power of A_K:
   A_K_power = A_K_power * A_K;
   
end

% divide by the alpha:
s = (1 / (1-alpha2)) * s;

end

