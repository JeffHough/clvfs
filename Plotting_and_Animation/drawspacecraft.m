%Just a test to try out the "fill3" for drawing a spacecraft
function [out] = drawspacecraft(size, O, d, LCone, C_BI, rT, color, rad_cone_close, rad_cone_far, NAME)

% good colour = [0.75 0.75 0.75]
q = size;

C_IB = C_BI';

X = [-q -q q q];
Y = [-q q q -q];
Z = [-q -q -q -q]; 

X2 = [-q -q q q];
Y2 = [-q q q -q];
Z2 = [q q q q]; 

X3 = [-q -q -q -q]; 
Y3 = [-q -q q q];
Z3 = [-q q q -q];

X4 = -1.*[-q -q -q -q]; 
Y4 = [-q -q q q];
Z4 = [-q q q -q];

X5 = Y;
Y5 = Z;
Z5 = X;

X6 = Y;
Y6 = -Z;
Z6 = X;

points = [X X2 X3 X4 X5 X6;
          Y Y2 Y3 Y4 Y5 Y6;
          Z Z2 Z3 Z4 Z5 Z6];

newpoints = C_IB*points + [rT(1) 0 0;
                           0 rT(2) 0;
                           0 0   rT(3)]*ones(3,24);

for i = 1:3
    if i == 1
        X = newpoints(i,1:4);
        X2 = newpoints(i,5:8);
        X3 = newpoints(i,9:12);
        X4 = newpoints(i,13:16);
        X5 = newpoints(i,17:20);
        X6 = newpoints(i,21:24);
    elseif i == 2
        Y = newpoints(i,1:4);
        Y2 = newpoints(i,5:8);
        Y3 = newpoints(i,9:12);
        Y4 = newpoints(i,13:16);
        Y5 = newpoints(i,17:20);
        Y6 = newpoints(i,21:24);
    else
        Z = newpoints(i,1:4);
        Z2 = newpoints(i,5:8);
        Z3 = newpoints(i,9:12);
        Z4 = newpoints(i,13:16);
        Z5 = newpoints(i,17:20);
        Z6 = newpoints(i,21:24); 
    end
end

d = C_IB*d; % + rT
O = C_IB*O; % Does not translate!
                       
gcf;
hold on;
s1 = fill3(X,Y,Z,color, "DisplayName",NAME);
s2 = fill3(X2,Y2,Z2,color, 'HandleVisibility','off');
s3 = fill3(X3,Y3,Z3,color, 'HandleVisibility','off');
s4 = fill3(X4,Y4,Z4,color, 'HandleVisibility','off');
s5 = fill3(X5,Y5,Z5,color, 'HandleVisibility','off');
s6 = fill3(X6,Y6,Z6,color, 'HandleVisibility','off');

% dUnit = d./sqrt(sum(d.^2));

dNorm = sqrt(sum(d.^2));

% g = Cone(d/2 - LCone/2*O + rT, d/2 + LCone/2*O + rT,[rad_cone_close, rad_cone_far],30,color,0,1);
g = Cone(d - dNorm/2*O - LCone/2*O + rT, d - dNorm/2*O + LCone/2*O + rT,[rad_cone_close, rad_cone_far],30,color,0,1, 'HandleVisibility','off');



% if thisIsTarget == 1
%     g = Cone(d - dNorm*O - LCone/2*O + rT, d - dNorm*O + LCone/2*O + rT,[rad_cone_close, rad_cone_far],30,color,0,1);
% end



xlabel 'x'
ylabel 'y'
zlabel 'z'
axis equal

out = [s1, s2, s3, s4, s5, s6, g];
end





