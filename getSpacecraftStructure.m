function [SpacecraftStructure] = getSpacecraftStructure()

% NOTE: THESE FIRST CHARACTERISTICS ARE FOR THE TARGET:

% PARAMETERS OF THE SETUP (NOT RELATED TO CLVF OR LVF DESIGN).   
SpacecraftStructure.o_prime = [1;0;0];
SpacecraftStructure.o_hat_prime = o_prime./sqrt(sum(o_prime.^2));
SpacecraftStructure.theta_d = 30*d2r;
SpacecraftStructure.d = [6;3;0]; % Might change later if it looks dumb.
SpacecraftStructure.dNorm = sqrt(sum(d.^2));
SpacecraftStructure.I = diag([35, 40, 55]);

% THESE CHARACTERISTICS ARE PURELY FOR PLOTTING/ANIMATION:

SpacecraftStructure.sizC = 2; % Size of the chaser.
SpacecraftStructure.sizT = 2; % Size of the target.

SpacecraftStructure.highConeWidth = 2*(sizT-d(2));
SpacecraftStructure.lowConeWidth = 0.5*highConeWidth;
SpacecraftStructure.coneHeight = (d(1)-sizC-sizT);

% CHASER SPACECRAFT MASS:
SpacecraftStructure.m = 10;

end

