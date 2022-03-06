function [d2Cdp2LVF] = costHessianLVF(W_LVF, vmax, alphap, s, dSdp,fact,theta_d,wandamax,wmax, mu)
%COSTHESSIANLVF 

    % Break W out of its vector:
    
        % For time:
            W_t_vmax = W_LVF(1);

        % For fuel:
            W_f_alphap = W_LVF(2);
            W_f_vmax = W_LVF(3);

        % For risk:
            W_r = W_LVF(4);
            
    % Now, get the individual Hessians:
    
        % For the time term:
            d2ctdp2 = W_t_vmax*[2/vmax              -2*alphap/vmax^2;
                                -2*alphap/vmax^2    2*alphap^2/vmax^3];
                            
        % For the fuel term:
            d2cfdp2 = 2*diag([W_f_alphap*(1+wandamax)^2, W_f_vmax]);
            
%         % For the risk term:
%             d2crdp2 = W_r*[2*vmax^2/alphap^3    -2*vmax/alphap^2;
%                           -2*vmax/alphap^2     2/alphap];

        % v_max no longer considered in the risk term.
            d2crdp2 = W_r*[2/alphap^3    0;
                          0              0];
                      
    % Total Hessian for performance:
        d2Cp_dp2 = d2ctdp2+d2cfdp2+d2crdp2;
        
    % Now, get the Hessian for the constraint cost:
        % First, add up the dSdp's from before:
        
        constraintHessian = zeros(2,2);
        
        for k = 1:numel(s) % Three constraints!
            constraintHessian = constraintHessian + dSdp(k,:)'*dSdp(k,:);
        end
        
    % Lastly, add in the Hessian from the upper acceleration constraint:
        secondDerPart = -hess_p_amaxLVF(alphap,fact,theta_d,vmax)/s(3);
        
    % Add up to get the total Hessian:
        d2Cdp2LVF = mu*d2Cp_dp2 + (constraintHessian - secondDerPart)./numel(s); % MINUS THE SECOND PART?

end

