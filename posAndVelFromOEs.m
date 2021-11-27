function [rI, vI] = posAndVelFromOEs(a, e, inc, RAAN, per, theta, MU)

    % First, getting distance "r" at this instant:
        r = a*(1-e^2)/(1+e*cos(theta));

    % Next, get the perifocal frame coordinates:
        rP = [r*cos(theta);
              r*sin(theta);
                  0      ];
              
    % Next, get the rotation matrix from perifocal into ECI:
        C_PI = C3(per)*C1(inc)*C3(RAAN);
        C_IP = C_PI';
        
    % Get rI
        rI = C_IP*rP;
        
    % Get the semi-latus rectum (p)
        p = a*(1-e^2);
        
    % Get perifocal frame velocity:
        vP = [-sqrt(MU/p)*sin(theta);
              sqrt(MU/p)*(e + cos(theta));
                         0              ];

    % Get the inertial velocity:
        vI = C_IP*vP;
end

