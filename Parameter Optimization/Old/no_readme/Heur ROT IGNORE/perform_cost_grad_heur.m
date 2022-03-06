function dCdp_heur = perform_cost_grad_heur(W,alpha,b,ka,kc,r,v,wandamax)
%PERFORM_COST_GRAD_HEUR
%    DCDP_HEUR = PERFORM_COST_GRAD_HEUR(W_F,W_J,W_T,ALPHA,B,KA,KC,RX,RY,RZ,VX,VY,VZ,WANDAMAX)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    29-Mar-2021 22:45:16

W_t = W(1);
W_f = W(2);
W_j = W(3);

rx = r(1);
ry = r(2);
rz = r(3);

vx = v(1);
vy = v(2);
vz = v(3);


t2 = rx.^2;
t3 = ry.^2;
t4 = rz.^2;
t5 = 1.0./ka.^2;
t6 = 1.0./kc.^2;
t7 = t2+t3+t4;
t8 = 1.0./sqrt(t7);
t9 = kc.*rx.*t8;
t10 = kc.*ry.*t8;
t11 = kc.*rz.*t8;
t12 = t9+vx;
t13 = t10+vy;
t14 = t11+vz;
dCdp_heur = [-(W_j.*1.0./b.^2)./t6+(W_t.*b.*2.0)./kc;W_f.*(((rx.*t8.*t12.*2.0+ry.*t8.*t13.*2.0+rz.*t8.*t14.*2.0).*1.0./sqrt(t12.^2+t13.^2+t14.^2))./2.0+1.0)-W_t.*(b.^2.*t6+t6./t8)+(W_j.*kc.*2.0)./b;W_f.*(pi./2.0-alpha.^2.*t5.*wandamax.*(1.3e+1./2.0e+1))-W_t.*alpha.*t5.*(1.3e+1./2.0)];
