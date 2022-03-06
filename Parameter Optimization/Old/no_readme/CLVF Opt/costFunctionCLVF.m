function C = costFunctionCLVF(b, kc, ka, s, W_CLVF, mu)

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

    % COMPUTE THE COST:
    
        % Time cost:
%             Ct = W_t_kc*b^2/kc + W_t_ka*b^2/ka;
            Ct = W_t_kc/kc + W_t_ka/ka + W_t_b*b^2;

        % Fuel cost:
            Cf = W_f_kc*kc^2 + W_f_ka*ka^2;

        % Choppy cost:
            Cc = W_c/b;

        % Constraint cost:
            C_constraints = sum(-log(s));

        % Total cost:
            C = mu*(Ct+Cf+Cc) + C_constraints/numel(s);
end

