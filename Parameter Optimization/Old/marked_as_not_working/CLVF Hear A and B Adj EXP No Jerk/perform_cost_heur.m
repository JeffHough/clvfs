function C = perform_cost_heur(W,alpha,b,ka,kc,r,v,wandamax,wmax)
%PERFORM_COST_HEUR
%    C = PERFORM_COST_HEUR(W_F,W_T,ALPHA,B,KA,KC,RX,RY,RZ,VX,VY,VZ,WANDAMAX,WMAX)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    13-Sep-2021 23:00:22

W_t = W(1);
W_f = W(2);

rx = r(1);
ry = r(2);
rz = r(3);

vx = v(1);
vy = v(2);
vz = v(3);

t2 = alpha.^2;
t3 = rx.^2;
t4 = ry.^2;
t5 = rz.^2;
t6 = 1.0./ka;
t7 = 1.0./kc;
t8 = t3+t4+t5;
t9 = 1.0./sqrt(t8);
C = W_t.*(t7.*(b.*2.0+b.^2./2.0+1.0./2.0)+alpha.*t6.*(1.3e+1./2.0)+t7./t9)+W_f.*(kc+ka.*pi+sqrt((vx+kc.*rx.*t9).^2+(vy+kc.*ry.*t9).^2+(vz+kc.*rz.*t9).^2)+t7.*wmax.*exp(b)+wmax.*pi.*sqrt(t2+t2.*t7.^2.*(ka+alpha.*wmax+1.0./t6.^2./4.0+1.0./4.0).^2).*2.0+t2.*t6.*wandamax.*1.3e+1);
