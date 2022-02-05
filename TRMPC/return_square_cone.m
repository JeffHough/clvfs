function [A_cone, b_cone] = return_square_cone(ha, d)
%% DESCRIPTION:
% returns a square cone approximation.

%% INPUTS:
% ha - the half angle of the square cone
% d_B - the origin of the square cone in the body frame

%% OUTPUTS:
% A_cone - a matrix describing the "cone" in the body-fixed frame.
% b_cone - the cone inequality matrix:

% v^T * (r - d) >= 0
% v^T * r       >= v^T * d

% Give four unit vectors (one for each side of the cone)
py = [0;1;0];
my = -py;
pz = [0;0;1];
mz = -pz;

% Now, rotate these vectors by the correct half angles:
py = R3(-ha) * py;
my = R3(ha) * my;
pz = R2(ha) * pz;
mz = R2(-ha) * mz;

% Now we can give the description of the cone as seen from the body frame:
A_cone = [py' ; my' ; pz' ; mz'];
b_cone = A_cone * d;


end

