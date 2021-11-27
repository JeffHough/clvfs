function [dCdp, dSdp] = costGradientCLVF_heur(alpha,b,ka,kc, ...
    W_CLVF,wmax,wandamax,rC_T0, vC_T0, s, mu, r_factor, W_s)
    
    % Break W out of its vector:
    
%         % For time:
%             W_t = W_CLVF(1);
%             W_f = W_CLVF(2);
%             W_j = W_CLVF(3);
%             W_s == for the "soft" penalty.
   
    % COST GRADIENT FOR PERFORMANCE:
        dCp_dp = perform_cost_grad_heur(W_CLVF,alpha,b,ka,kc,rC_T0,vC_T0,wandamax);
        
    % COST GRADIENT FOR SOFT CONSTRAINT:
        powerPenalty = -kc*r_factor*alpha + ka*alpha + wmax*alpha^2;
        dPower_dp = [0, -r_factor*alpha, alpha];
%         penalty = W_s*exp(powerPenalty);
        
        dPenalty_dp = W_s*exp(powerPenalty)*dPower_dp;
            
    % COST GRADIENT FOR SLACKS:  
        dSdp = zeros(4, 3);
        
        dSdp(1,1) = 1/s(1);
        dSdp(2,2) = 1/s(2);
        dSdp(3,3) = 1/s(3);
        
        dSdp(4,:) = -grad_p_amaxCLVF_quad(alpha,b,ka,kc,wmax)'./s(4);
        
    % Total slack cost gradient:
        dConstraints_dp = transpose(sum(dSdp));
        
    % TOTAL cost gradient:
        dCdp = mu*(dCp_dp + dPenalty_dp) - dConstraints_dp/numel(s);
end

