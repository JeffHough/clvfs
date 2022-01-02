function [P, z] = return_inequality_con_mat_and_vec(u_con_mat, u_con_vec, x_con_mat, x_con_vec, Np, Gf, G_vec)

    %% NEED TO MAKE THIS MORE SPECIFIC TO MY PROBLEM WITH THE MOVING TARGET... 
     % SHOULD PASS IN THE MOVING DOCKING PORT INFO AS MATRIX ON ITS OWN?

    % Get the total size of the U and X vectors:
    u_dim = size(u_con_mat, 2);
    x_dim = size(x_con_mat, 2);
    
    % Get the total "constraint height" for each vector:
    u_height = size(u_con_mat, 1);
    x_height = size(x_con_mat, 1);
    
    % Preallocate size for the overall constraint matrix and vector:
    totalWidth = Np* (u_dim + x_dim);    
    totalHeight = Np * (u_height + x_height);
    
    % First, check if we are including the last point or not:
    if isempty(Gf)
       P = zeros(totalHeight, totalWidth);
       z = zeros(totalHeight, 1);
    else
        G_size = size(Gf);
        
        P = zeros(totalHeight + G_size(1), totalWidth);
        z = zeros(totalHeight + G_size(1), 1);
    end
    
    % Create a single instance of the combined U and X constraint matrix:
    M_mat = [u_con_mat, zeros(u_height, x_dim);
             zeros(x_height, u_dim), x_con_mat];

    M_vec = [u_con_vec;x_con_vec];
         
    % Loop through, filling in the values:
    iHeight = 1;
    iWidth = 1;
    
    for iMat = 1:Np
       %  What is the end?
       iHeightEnd = iHeight + (u_height + x_height - 1);
       iWidthEnd = iWidth + (u_dim + x_dim - 1);
       
       % Fill in the matrix and the vector:
       P(iHeight:iHeightEnd, iWidth:iWidthEnd) = M_mat;
       z(iHeight:iHeightEnd) = M_vec;
       
       % Increment:
       iHeight = iHeightEnd + 1;
       iWidth = iWidthEnd + 1;
        
    end
    
    if ~isempty(Gf)
        P(end-G_size(1) + 1:end, end-G_size(2) + 1:end) = Gf;
        z(end-G_size(1) + 1:end) = G_vec;
    end
    
    
end

