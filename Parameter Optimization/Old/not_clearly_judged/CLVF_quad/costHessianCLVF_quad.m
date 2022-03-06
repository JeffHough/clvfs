function [d2Cdp2CLVF] = costHessianCLVF_quad(alpha,b,ka,kc, ...
                                                W_CLVF, s, dSdp,wmax, mu)
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
            
    % Now, get the individual Hessians:
    
        % For the time term:
%             d2ctdp_PART1 = W_t_kc*  [2/kc        -2*b/kc^2    0;
%                                      -2*b/kc^2    2*b^2/kc^3  0
%                                                 zeros(1,3)];
%                                             
%             d2ctdp_PART2 = W_t_ka*  [2/ka       0         -2*b/ka^2 ;
%                                               zeros(1,3)             
%                                      -2*b/ka^2    0       2*b^2/ka^3];

            d2ctdp_PART1 = W_t_kc*  [0      0    0;
                                     0    2/kc^3  0
                                      zeros(1,3)];
                                            
            d2ctdp_PART2 = W_t_ka*  [0       0       0 ;
                                        zeros(1,3)             
                                     0        0   2/ka^3];
                                 
            d2ctdp_PART3 = 2*diag([W_t_b, 0, 0]);
                                 
            d2ctdp2 = d2ctdp_PART1 + d2ctdp_PART2 + d2ctdp_PART3;
                            
        % For the fuel term:
            d2cfdp2 = 2*diag([0, W_f_kc, W_f_ka]);
            
        % For the choppy term:
%             d2ccdp2 = W_c * [2*kc^2/b^3     -2*kc/b^2       0
%                              -2*kc/b^2         2/b          0
%                                              zeros(1,3)      ];

            d2ccdp2 = W_c * [2/b^3     0       0
                               0       0       0
                                  zeros(1,3)      ];
                      
    % Total Hessian for performance:
        d2Cp_dp2 = d2ctdp2+d2cfdp2+d2ccdp2;
        
    % Now, get the Hessian for the constraint cost:
        % First, add up the dSdp's from before:
        
        constraintHessian = zeros(3);
        
        for k = 1:numel(s) % There are 4 constraints!
            constraintHessian = constraintHessian + dSdp(k,:)'*dSdp(k,:);
        end
        
    % Lastly, add in the Hessian from the upper acceleration constraint:
        secondDerPart = -hess_p_amaxCLVF_quad(alpha,b,ka,kc,wmax)./s(end);
        
    % Add up to get the total Hessian:
        d2Cdp2CLVF = mu*d2Cp_dp2 + (constraintHessian - secondDerPart)/numel(s); % MINUS THE SECOND PART?

end













