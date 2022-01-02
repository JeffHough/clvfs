function [A_cone] = return_square_cone(ha)
%% DESCRIPTION:
% returns a square cone approximation.

%% INPUTS:
% ha - the half angle of the square cone
% d_B - the origin of the square cone in the body frame
% C_BI - the orientation of the target "B" compared to ineratial "I".

%% OUTPUTS:
% A_cone - a matrix describing the "cone".

% Give four unit vectors (one for each side of the cone)
py = [0;1;0];
my = -py;
pz = [0;0;1];
mz = -pz;

% Now, rotate these vectors by the correct half angles:
py = math.R3(-ha) * py;
my = math.R3(ha) * my;
pz = math.R2(ha) * pz;
mz = math.R2(-ha) * mz;

% Now we can give the description of the cone as seen from the body frame:
A_cone = [py' ; my' ; pz' ; mz'];

end

