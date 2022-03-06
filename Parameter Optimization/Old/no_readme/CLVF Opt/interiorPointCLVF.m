function [b, kc, ka] = interiorPointCLVF(b0, kc0, ka0, a, a_max, w_max, rotNorm, W_CLVF, mu, tol, gamma, beta, muFact, muLimit)

%             tol = 10^-8;
%             mu = 100; % My weighting parameter for the perf vs. constraints.
%             gamma = 1; % My damping factor
%             beta = 0.8; % My reduction factor.

            % LEGEND OF WEIGHTS:

%         For time:
%             W_t_kc = W_CLVF(1);
%             W_t_ka = W_CLVF(2);
% 
%         For fuel:
%             W_f_kc = W_CLVF(3);
%             W_f_ka = W_CLVF(4);
% 
%         For chopiness:
%             W_c = W_CLVF(5);
%             W_t_b = W_CLVF(6); (NOT VERY IMPORTANT, KEEPS B FROM BEING
%             MASSIVE THOUGH).

%             W_CLVF = [100, 100, 10, 10, 100, 1];

            while mu < muLimit
                [b, kc, ka] = newtonStepCLVF(b0, kc0, ka0, a, a_max, w_max, rotNorm, W_CLVF, mu, tol, gamma, beta);
                mu = muFact*mu;
                b0 = b;
                kc0 = kc;
                ka0 = ka;
            end

end

