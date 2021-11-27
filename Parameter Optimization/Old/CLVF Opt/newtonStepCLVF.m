function [b, kc, ka] = newtonStepCLVF(b0, kc0, ka0, alpha, amax, wmax, wandamax, W_CLVF, mu, tol, gamma, beta)

    % FOR A GIVEN MU VALUE WHICH WILL CHANGE IN OUTER LOOP.
    
    % Get the slack variables:
        s0 = computeSlacksCLVF(b0, kc0, ka0, alpha,... 
                                amax, wmax, wandamax);
                            
        if isempty(s0)
           disp("Invalid start point");
           return;
        end
                            
        C0 = costFunctionCLVF(b0,kc0, ka0, s0, W_CLVF, mu);

    cnt = 1; % Counts the step we're on.

    while 1 % Go on forever.
    
        % First, check gradient:
            [dCdp, dSdp0] = costGradientCLVF(alpha,b0,ka0,kc0, ...
                                                W_CLVF,wmax, s0, mu);

            stdGrad = sqrt(sum(dCdp.^2));
            
            disp(stdGrad);

        % Check if we're already good enough!
        
        if stdGrad <= tol % We ARE good enough!!
            
            if cnt == 1 % The first step:
                % We're already at a minimum!
                b = b0;
                kc = kc0;
                ka = ka0;
            end
            
            disp("Found minimum");
            break;
            
        else % We are NOT good enough :(
            
            % Compute Hessian, the compute step.
                dC2dp2 = costHessianCLVF(alpha,b0,ka0,kc0, W_CLVF, s0, dSdp0,wmax, mu);
                xStep = -dC2dp2\dCdp;
            
            % Take the step:
                newParams = [b0;kc0;ka0] + gamma*xStep;
                b = newParams(1);
                kc = newParams(2);
                ka = newParams(3);
                
            % Recompute the slack variable - if we left the bounds, then
            % dampen!
                s = computeSlacksCLVF(b, kc, ka, alpha,... 
                                        amax, wmax, wandamax);
                                    
                C = costFunctionCLVF(b,kc, ka, s, W_CLVF, mu);
                
            while isempty(s)% || C >= C0
                % Smaller step. Try it all again.
                    xStep = beta*xStep; 
                % Take the step:
                    newParams = [b0;kc0;ka0] + gamma*xStep;
                    b = newParams(1);
                    kc = newParams(2);
                    ka = newParams(3);
                
            % Recompute the slack variable - if we left the bounds, then
            % dampen!
                s = computeSlacksCLVF(b, kc, ka, alpha,... 
                                        amax, wmax, wandamax);
                C = costFunctionCLVF(b,kc, ka, s, W_CLVF, mu);
                                   
            end
            
        end

        % RESET:
            s0 = s; b0 = b; kc0 = kc; ka0 = ka; C0 = C;
            cnt = cnt+1;
    end
    
end

