function [dCdp, dSdp] = costGradientCLVF(alpha,b,ka,kc, ...
    W_CLVF,wmax, s, mu)
    
    % Break W out of its vector:
    
        % For time:
            W_t_kc = W_CLVF(1);
            W_t_ka = W_CLVF(2);
            W_t_b = W_CLVF(6); % Not very important.

        % For fuel:
            W_f_kc = W_CLVF(3);
            W_f_ka = W_CLVF(4);

        % For chopiness:
            W_c = W_CLVF(5);
            
    % COST GRADIENT FOR PERFORMANCE:
%         % Time cost gradient:
%             dCt_db = 2*W_t_kc* b/kc + 2*W_t_ka* b/ka;
%             dCt_dkc = -W_t_kc * b^2/kc^2;
%             dCt_dka = -W_t_ka * b^2/ka^2;

            dCt_db = 2*W_t_b*b;
            dCt_dkc = -W_t_kc/kc^2;
            dCt_dka = -W_t_ka/ka^2;
            
        % Fuel cost gradient:
%             dCf_db = 0;
            dCf_dkc = 2*W_f_kc*kc;
            dCf_dka = 2*W_f_ka*ka;
            
        % Choppiness cost gradient:
%             dCc_db = -W_c*kc^2/b^2;
%             dCc_dkc = 2*W_c*kc/b;

            dCc_db = -W_c/b^2;
            dCc_dkc = 0;
            
        % Total performance cost gradient:
            dCp_dp = [dCt_db+dCc_db;
                      dCt_dkc + dCf_dkc + dCc_dkc;
                      dCt_dka + dCf_dka];
                   
            
    % COST GRADIENT FOR SLACKS:  
        dSdp = zeros(4, 3);
        
        dSdp(1,1) = 1/s(1);
        dSdp(2,2) = 1/s(2);
        dSdp(3,3) = 1/s(3);
        
        dSdp(4,:) = -grad_p_amaxCLVF(alpha,b,ka,kc,wmax)'./s(4);        
    % Total slack cost gradient:
        dConstraints_dp = transpose(sum(dSdp));
        
    % TOTAL cost gradient:
        dCdp = mu*dCp_dp - dConstraints_dp/numel(s);
end

