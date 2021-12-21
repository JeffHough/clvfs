function [C] = return_cost_matrix(u_con_mat, x_con_mat, Np, Q_final)

    % First, get the size of the input and state vectors:
    u_dim = size(u_con_mat,2);
    x_dim = size(x_con_mat,2);

    % Overall, there are Np-1 u-vectors and Np-1 x-vectors:
    totalSize = (Np-1)*(u_dim + x_dim);
    C = zeros(totalSize);

    % Assign a combined input-state block:
    Z = [u_con_mat,             zeros(u_dim, x_dim);
         zeros(x_dim, u_dim),           x_con_mat];

    % Loop through populating the cost matrix:
    iDiag = 1;
    while iDiag < totalSize
       iDiagEnd = iDiag + (u_dim + x_dim - 1); 
       C(iDiag:iDiagEnd, iDiag:iDiagEnd) = Z;
       iDiag = iDiagEnd + 1;
    end
    
    % replace just the last lower right corner:
    C(end-x_dim + 1 : end, end - x_dim + 1:end) = Q_final;
 
end

