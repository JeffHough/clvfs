function [constraint_vector_mpc] = return_reduced_constraints(premult_matrix, error_constraint_matrix,error_constraint_vector, nominal_constraint_vector)

options = optimoptions('linprog','Algorithm','interior-point', 'Display','off');

% preallocate size for the solution (equal to the number of rows in the premultiplication matrix):
bu = zeros(size(premult_matrix,1), 1);

for iRow = 1:numel(bu)
   % Get the f-vector:
   f = -premult_matrix(iRow, :)';
   
   % Get the constraint matrix...
   e_worst_case = linprog(f, error_constraint_matrix, error_constraint_vector, [], [], [], [], options);
   
   % compute the worst-case, and fill in bu:
   bu(iRow) = -f'*e_worst_case;
    
end

% Then, my input constraints for the MPC system are:
constraint_vector_mpc = nominal_constraint_vector - bu;

end

