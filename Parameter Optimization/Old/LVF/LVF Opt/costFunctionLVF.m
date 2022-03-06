function C = costFunctionLVF(alphap, vmax, s, W_LVF, mu, wandamax)

    % Break W out of its vector:
    
        % For time:
            W_t_vmax = W_LVF(1);

        % For fuel:
            W_f_alphap = W_LVF(2);
            W_f_vmax = W_LVF(3);

        % For risk:
            W_r = W_LVF(4);

    % COMPUTE THE COST:
    
        % Time cost:
            Ct = W_t_vmax*alphap^2/vmax;

        % Fuel cost:
            Cf = W_f_alphap*(1+wandamax)^2*alphap^2 +...
                W_f_vmax*vmax^2;

        % Risk cost:
            Cr = W_r * 1/alphap; % CHANGED TO A 1!! this is the main risk.

        % Constraint cost:
            C_constraints = sum(-log(s));

        % Total cost:
            C = mu*(Ct+Cf+Cr) + C_constraints/numel(s);
end

