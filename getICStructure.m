function [ICStructure] = getICStructure(SpacecraftStructure, MU)

    % Conversion from degrees to radians:
    d2r = pi/180;
    
    % Get the spacecraft data:
    J_z = SpacecraftStructure.J_z;
    J_transverse = SpacecraftStructure.J_transverse;
    m = SpacecraftStructure.m;
    

    % Set up the target orbit:
    ICStructure.OE.semiMajorAxis = 7500;
    ICStructure.OE.e = 0.01;
    ICStructure.OE.i = 0 * d2r;
    ICStructure.OE.RAAN = 0*d2r;
    ICStructure.OE.argOfPerigee = 0*d2r;
    ICStructure.OE.trueAnomaly = 0*d2r;

    % Get the initial position and velocity from the orbital elements:
    [ICStructure.RT_I0, ICStructure.VT_I0] = posAndVelFromOEs( ...
        ICStructure.OE.semiMajorAxis, ...
        ICStructure.OE.e, ...
        ICStructure.OE.i, ...
        ICStructure.OE.RAAN, ...
        ICStructure.OE.argOfPerigee, ...
        ICStructure.OE.trueAnomaly, ...
        MU...
    );

    % Declare the different initial condition options:
    ICStructure.rC_I0 = {};
    ICStructure.vC_I0 = {};
    ICStructure.CT_BI0 = {};
    ICStructure.wT_B0 = {};
    ICStructure.gamma = {};
    ICStructure.w_t = {};
    ICStructure.mu0 = {};
    ICStructure.theta0 = {};


    % Now, create the initial conditions from EASIEST to HARDEST case.
    ICStructure.rC_I0{1} = [300;-300;500]*10^-3;
    ICStructure.vC_I0{1} = [0;0;0]*10^-3;
    ICStructure.CT_BI0{1} = eye(3,3);
    ICStructure.wT_B0{1} = [0.2;0.05;-0.45] * d2r;
    ICStructure.gamma{1} = 30*pi/180;
    ICStructure.w_t{1} = 0 * d2r;
    ICStructure.mu0{1} = 0;
    ICStructure.theta0{1} = 0;

    ICStructure.rC_I0{2} = [900;-1500;-900]*10^-3;
    ICStructure.vC_I0{2} = [0;0;0]*10^-3;
    ICStructure.CT_BI0{2} = eye(3,3);
    ICStructure.wT_B0{2} = [-1.4;1.0;2.0] * d2r; 
    ICStructure.gamma{2} = 30*pi/180;
    ICStructure.w_t{2} = 2 * d2r;
    ICStructure.mu0{2} = 0;
    ICStructure.theta0{2} = 0;

    ICStructure.rC_I0{3} = [30;-50;-30]*10^-3;
    ICStructure.vC_I0{3} = [0;0;0]*10^-3;
    ICStructure.CT_BI0{3} = eye(3,3);
    ICStructure.wT_B0{3} = [-0.2; 0.8; 3.1]* d2r;
    ICStructure.gamma{3} = 30*pi/180;
    ICStructure.w_t{3} = 4 * d2r;
    ICStructure.mu0{3} = 0;
    ICStructure.theta0{3} = 0;

    ICStructure.rC_I0{4} = [300;-500;-300]*10^-3;
    ICStructure.vC_I0{4} = [30;-50;10]*10^-3;
    ICStructure.CT_BI0{4} = eye(3,3);
    ICStructure.wT_B0{4} = [-1.4;1.0;4.0]* d2r;
    ICStructure.gamma{4} = 30*pi/180;
    ICStructure.w_t{4} = 6 * d2r;
    ICStructure.mu0{4} = 0;
    ICStructure.theta0{4} = 0;

    ICStructure.rC_I0{5} = [50;-30;-30]*10^-3;
    ICStructure.vC_I0{5} = [10;-5.7;3.1]*10^-3;
    ICStructure.CT_BI0{5} = eye(3,3);
    ICStructure.wT_B0{5} = [1.0;-1.4;5.0] * d2r;
    ICStructure.gamma{5} = 30*pi/180;
    ICStructure.w_t{5} = 8 * d2r;
    ICStructure.mu0{5} = 0;
    ICStructure.theta0{5} = 0;

    ICStructure.rC_I0{6} = [5000;-3000;-3000]*10^-3;
    ICStructure.vC_I0{6} = [-2;3;1.0]*10^-3;
    ICStructure.CT_BI0{6} = eye(3,3);
    ICStructure.wT_B0{6} = [1.0;-3.4;7.0] * d2r;
    ICStructure.gamma{6} = 30*pi/180;
    ICStructure.w_t{6} = 10 * d2r;
    ICStructure.mu0{6} = 0;
    ICStructure.theta0{6} = 0;

    ICStructure.rC_I0{7} = [3000;-3000;5000]*10^-3;
    ICStructure.vC_I0{7} = [0;0;0]*10^-3;
    ICStructure.CT_BI0{7} = eye(3,3);
    ICStructure.wT_B0{7} = [1.0;-2.4;10.0] * d2r;
    ICStructure.gamma{7} = 30*pi/180;
    ICStructure.w_t{7} = 12 * d2r;
    ICStructure.mu0{7} = 0;
    ICStructure.theta0{7} = 0;

    for icSelection = 1:7
        % Calculate remaining values:
        ICStructure.h_t{icSelection} = J_transverse * ICStructure.w_t{icSelection};
        ICStructure.h{icSelection} = ICStructure.h_t{icSelection} / sin(ICStructure.gamma{icSelection});
        ICStructure.h_z{icSelection} = ICStructure.h{icSelection} * cos(ICStructure.gamma{icSelection});
        ICStructure.w_z{icSelection} = ICStructure.h_z{icSelection} / J_z;
    end

end


