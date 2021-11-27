function C = perform_cost_heur(W,alpha,b,ka,kc,r,v,wandamax,wmax)
%PERFORM_COST_HEUR
%    C = PERFORM_COST_HEUR(W_F,W_T,ALPHA,B,KA,KC,RX,RY,RZ,VX,VY,VZ,WANDAMAX,WMAX)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    19-Sep-2021 15:52:38

W_t = W(1);
W_f = W(2);

rx = r(1);
ry = r(2);
rz = r(3);

vx = v(1);
vy = v(2);
vz = v(3);

t2 = alpha.^2;
t3 = b.^2;
t4 = ka.^2;
t5 = rx.^2;
t6 = ry.^2;
t7 = rz.^2;
t8 = wmax.^2;
t9 = 1.0./ka;
t10 = 1.0./kc;
t11 = t5+t6+t7;
t12 = 1.0./sqrt(t11);
C = W_t.*(alpha.*t9.*(1.3e+1./2.0)+t10./t12+t10.*(b.*2.0+t3./2.0+1.0./2.0))+W_f.*(kc+(ka.*pi)./2.0+sqrt((vx+kc.*rx.*t12).^2+(vy+kc.*ry.*t12).^2+(vz+kc.*rz.*t12).^2)+wmax.*pi.*sqrt(t2+t2.*t10.^2.*(ka+t4./4.0+alpha.*wmax+1.0./4.0).^2)+t2.*t9.*wandamax.*(1.3e+1./2.0)+(t10.*pi.*(alpha.*t4.*4.0+b.^3.*t8+alpha.*t3.*t8.*8.0).*(2.0./3.0))./alpha);
