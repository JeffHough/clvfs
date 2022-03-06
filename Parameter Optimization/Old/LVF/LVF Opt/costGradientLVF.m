function [dCdp, dSdp] = costGradientLVF(alphap, vmax, ...
    W_LVF, fact,theta_d, wandamax, wmax, s, mu)
    % Break W out of its vector:
    
        % For time:
            W_t_vmax = W_LVF(1);

        % For fuel:
            W_f_alphap = W_LVF(2);
            W_f_vmax = W_LVF(3);

        % For risk:
            W_r = W_LVF(4);
            
    % COST GRADIENT FOR PERFORMANCE:
        % Time cost gradient:
            dCt_dalphap = 2*W_t_vmax*alphap/vmax;
            dCt_dvmax = -W_t_vmax*alphap^2/vmax^2;
            
        % Fuel cost gradient:
            dCf_dalphap = 2*W_f_alphap*(1+wandamax)^2*alphap;
            dCf_dvmax = 2*W_f_vmax*vmax;
    
        % Risk cost gradient:
%             dCr_dalphap = -W_r*vmax^2/alphap^2;
%             dCr_dvmax = 2*W_r*vmax/alphap;

        % Taking v_max out of the picture for this.
            dCr_dalphap = -W_r/alphap^2;
            dCr_dvmax = 0;
            
        % Total performance cost gradient:
            dCp_dp = [dCt_dalphap + dCf_dalphap + dCr_dalphap;
                      dCt_dvmax + dCf_dvmax + dCr_dvmax];
                   
            
    % COST GRADIENT FOR SLACKS:  
        dSdp = zeros(3, 2);
        
        dSdp(1,1) = 1/s(1);
        dSdp(2,2) = 1/s(2);
        
        dSdp(3,:) = -grad_p_amaxLVF(alphap,fact,theta_d,vmax,wandamax,wmax)'./s(3);
        
    % Total slack cost gradient:
        dConstraints_dp = transpose(sum(dSdp));
        
    % TOTAL cost gradient:
        dCdp = mu*dCp_dp - dConstraints_dp/numel(s);
end

