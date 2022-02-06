function [ICStructure] = getICStructure()

d2r = pi/180;

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
    ICStructure.OE.MU...
);

% Declare the different initial condition options:

ICStructure.rC_I0 = {};
ICStructure.vC_I0 = {};
ICStructure.CT_BI0 = {};
ICStructure.wT_B0 = {};

% Now, create the initial conditions from EASIEST to HARDEST case.
ICStructure.rC_I0(1) = [300;-300;500]*10^-3;
ICStructure.vC_I0(1) = [0;0;0]*10^-3;
ICStructure.CT_BI0(1) = eye(3,3);
ICStructure.wT_B0(1) = [0.2;0.05;-0.45] * d2r;

ICStructure.rC_I0(2) = [900;-1500;-900]*10^-3;
ICStructure.vC_I0(2) = [0;0;0]*10^-3;
ICStructure.CT_BI0(2) = eye(3,3);
ICStructure.wT_B0(2) = [-1.4;1.0;2.0] * d2r; 

ICStructure.rC_I0(3) = [30;-50;-30]*10^-3;
ICStructure.vC_I0(3) = [0;0;0]*10^-3;
ICStructure.CT_BI0(3) = eye(3,3);
ICStructure.wT_B0(3) = [-0.2; 0.8; 3.1]* d2r;

ICStructure.rC_I0(4) = [300;-500;-300]*10^-3;
ICStructure.vC_I0(4) = [30;-50;10]*10^-3;
ICStructure.CT_BI0(4) = eye(3,3);
ICStructure.wT_B0(4) = [-1.4;1.0;4.0]* d2r;

ICStructure.rC_I0(5) = [50;-30;-30]*10^-3;
ICStructure.vC_I0(5) = [10;-5.7;3.1]*10^-3;
ICStructure.CT_BI0(5) = eye(3,3);
ICStructure.wT_B0(5) = [1.0;-1.4;5.0] * d2r;

ICStructure.rC_I0(6) = [5000;-3000;-3000]*10^-3;
ICStructure.vC_I0(6) = [-2;3;1.0]*10^-3;
ICStructure.CT_BI0(6) = eye(3,3);
ICStructure.wT_B0(6) = [1.0;-3.4;7.0] * d2r;

ICStructure.rC_I0(7) = [3000;-3000;5000]*10^-3;
ICStructure.vC_I0(7) = [0;0;0]*10^-3;
ICStructure.CT_BI0(7) = eye(3,3);
ICStructure.wT_B0(7) = [1.0;-2.4;10.0] * d2r;

end

