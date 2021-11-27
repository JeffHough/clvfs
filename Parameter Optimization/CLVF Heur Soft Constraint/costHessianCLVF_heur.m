function [d2Cdp2CLVF] = costHessianCLVF_heur(alpha,b,ka,kc,rC_T0,vC_T0,wandamax, ...
                                                W_CLVF, s, dSdp, wmax, mu, r_factor, W_s)
    % Break W out of its vector:

%             W_t = W_CLVF(1);
%             W_f = W_CLVF(2);
%             W_j = W_CLVF(3);
%             W_s == soft constraint.
    

    % Now, get the individual Hessians:
       d2Cp_dp2 =  perform_cost_hess_heur(W_CLVF,alpha,b,ka,kc,rC_T0,vC_T0,wandamax);
        
    % Now, get the Hessian for the constraint cost:
        % First, add up the dSdp's from before:
        
        constraintHessian = zeros(3);
        
        for k = 1:numel(s) % There are 4 constraints!
            constraintHessian = constraintHessian + dSdp(k,:)'*dSdp(k,:);
        end
        
    % Lastly, add in the Hessian from the upper acceleration constraint:
        secondDerPart = -hess_p_amaxCLVF_quad(alpha,b,ka,kc,wmax)./s(end);
        
    % COST GRADIENT FOR SOFT CONSTRAINT:
        powerPenalty = -kc*r_factor*alpha + ka*alpha + wmax*alpha^2;
        dPower_dp = [0, -r_factor*alpha, alpha];
%         penalty = W_s*exp(powerPenalty);
        
        d2Penalty_dp2 = W_s*exp(powerPenalty)*(dPower_dp'*dPower_dp);
        
    % Add up to get the total Hessian:
        d2Cdp2CLVF = mu*(d2Cp_dp2+d2Penalty_dp2) + (constraintHessian - secondDerPart)/numel(s); % MINUS THE SECOND PART?

end













