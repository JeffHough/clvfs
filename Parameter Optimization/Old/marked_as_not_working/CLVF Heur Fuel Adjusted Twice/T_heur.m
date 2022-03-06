function T_est = T_heur(alpha,b,ka,kc,r)
%T_HEUR
%    T_EST = T_HEUR(ALPHA,B,KA,KC,RX,RY,RZ)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    30-Aug-2021 17:55:38

rx = r(1);
ry = r(2);
rz = r(3);


t2 = 1.0./kc;
T_est = t2.*sqrt(rx.^2+ry.^2+rz.^2)+(alpha.*1.3e+1)./ka+b.^2.*t2;