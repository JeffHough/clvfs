function [C_BI_Np, omega_Np] = return_C_BI_from_axisym(mu_0, gamma_0, theta_0, t_vec, J_z, J_transverse, omega_z, omega_t)


    C_BI_Np = zeros(3,3,numel(t_vec));
    omega_Np = zeros(3,1,numel(t_vec));

    % First, solve for captital omega:
    mu_dot = (J_transverse - J_z)/J_transverse * omega_z;
    
    % Solve for the rate of precession:
    theta_dot = omega_t/sin(gamma_0);
    
    % Solve for the upcoming angles:
    ONE = ones(size(t_vec));
    
    mu_future = mu_0 * ONE + mu_dot*t_vec;
    theta_future = theta_0 * ONE + theta_dot * t_vec;
    
    % solve for the upcoming matrices and angular velocities:
    for iTime = 1:numel(t_vec)
        this_mu = mu_future(iTime);
        this_theta = theta_future(iTime);
        
       C_BI_Np(:,:,iTime) =  C3(this_mu)*C1(gamma_0)*C3(this_theta);
       omega_Np = [theta_dot*sin(gamma_0)*sin(this_mu);
                   theta_dot*sin(gamma_0)*cos(this_mu);
                   mu_dot + theta_dot*cos(gamma_0)];
       
    end

end

