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

% this is simply solving the x_squiggle and the x_bar, then adding the
% answers together.

s = solve_x_squiggle_linear_program(G_i, s1, alpha1, A_L, L, A_w, b_w, A_v, b_v) + solve_x_bar_linear_program(G_i, s1, s2, alpha1, alpha2, A_L, C, L, A_K, A_w, b_w, A_v, b_v);


end

