function [s] = computeSlacksLVF(alphap, vmax,... 
    amax, wmax, wandamax, dockingPortNorm, theta_d, fact)

    % Returns slack variables if all constraints are met. Otherwise,
    % returns an empty vector.
    
    s = zeros(3,1);
    
    s(1) = alphap; s(2) = vmax;
    
    paramsLVF = [vmax, alphap];
    rotStuffLVF  = [dockingPortNorm, wandamax, wmax, theta_d, fact];

    s(3) = amax - a_max_LVF3D(paramsLVF, rotStuffLVF);
    
    if ~isempty(find(s<=0, 1))
        s = []; % Give back blank vector.
    end
end

