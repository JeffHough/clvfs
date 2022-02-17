function [SpacecraftStructure] = getSpacecraftStructure()

% Conversion for degrees into radians:
d2r = pi/180;

% NOTE: THESE FIRST CHARACTERISTICS ARE FOR THE TARGET:

% PARAMETERS OF THE SETUP (NOT RELATED TO CLVF OR LVF DESIGN).   
SpacecraftStructure.theta_d = 30*d2r;
SpacecraftStructure.d = [6;3;0]; % Might change later if it looks dumb.
SpacecraftStructure.dNorm = sqrt(sum(SpacecraftStructure.d.^2));
SpacecraftStructure.I = diag([35, 40, 55]);

% THESE CHARACTERISTICS ARE PURELY FOR PLOTTING/ANIMATION:

SpacecraftStructure.sizC = 2; % Size of the chaser.
SpacecraftStructure.sizT = 2; % Size of the target.

SpacecraftStructure.highConeWidth = 2*(SpacecraftStructure.sizT-SpacecraftStructure.d(2));
SpacecraftStructure.lowConeWidth = 0.5*SpacecraftStructure.highConeWidth;
SpacecraftStructure.coneHeight = (SpacecraftStructure.d(1)-SpacecraftStructure.sizC-SpacecraftStructure.sizT);

% CHASER SPACECRAFT MASS:
SpacecraftStructure.m = 10;

% THESE CHARACTERISTICS ARE FOR IF THE TARGET SPACECRAFT IS AXI-SYMETRIC:
SpacecraftStructure.J_transverse = 2;
SpacecraftStructure.J_z = 5;

% USING THE SQUARE CONE CONSTRAINT:
% Pick a random rotation for the o_hat_prime vector:
SpacecraftStructure.C_CB = C2(pi/7);

% Get o_hat_prime as this rotation from x (has to be from x based
% on how the cone is constructed in return_square_cone function):
SpacecraftStructure.o_hat_prime = SpacecraftStructure.C_CB' * [1;0;0];

[SpacecraftStructure.A_cone, SpacecraftStructure.b_cone] = return_square_cone(...
    SpacecraftStructure.theta_d, ...
    SpacecraftStructure.d, ...
    SpacecraftStructure.C_CB ...
);

end

