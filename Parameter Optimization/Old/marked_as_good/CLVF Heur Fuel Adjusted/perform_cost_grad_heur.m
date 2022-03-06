function dCdp_heur = perform_cost_grad_heur(W,alpha,b,ka,kc,r,v,wandamax,wmax)
%PERFORM_COST_GRAD_HEUR
%    DCDP_HEUR = PERFORM_COST_GRAD_HEUR(W_F,W_J,W_T,ALPHA,B,KA,KC,RX,RY,RZ,VX,VY,VZ,WANDAMAX,WMAX)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    29-Aug-2021 21:42:32

W_t = W(1);
W_f = W(2);
W_j = W(3);

rx = r(1);
ry = r(2);
rz = r(3);

vx = v(1);
vy = v(2);
vz = v(3);

t2 = alpha.*wmax;
t3 = alpha.^2;
t4 = ka.^2;
t5 = rx.^2;
t6 = ry.^2;
t7 = rz.^2;
t9 = 1.0./kc.^2;
t8 = 1.0./t4;
t10 = t4./4.0;
t11 = t5+t6+t7;
t12 = ka+t2+t10+1.0./4.0;
t13 = 1.0./sqrt(t11);
t14 = t12.^2;
t15 = kc.*rx.*t13;
t16 = kc.*ry.*t13;
t17 = kc.*rz.*t13;
t18 = t15+vx;
t19 = t16+vy;
t20 = t17+vz;
t21 = t3.*t9.*t14;
t22 = t3+t21;
t23 = 1.0./sqrt(t22);
dCdp_heur = [-(W_j.*1.0./b.^2)./t9+(W_t.*b.*2.0)./kc;-W_t.*(b.^2.*t9+t9./t13)+W_f.*(((rx.*t13.*t18.*2.0+ry.*t13.*t19.*2.0+rz.*t13.*t20.*2.0).*1.0./sqrt(t18.^2+t19.^2+t20.^2))./2.0-alpha.*1.0./kc.^3.*t2.*t14.*t23.*pi+1.0)+(W_j.*kc.*2.0)./b;W_f.*(pi./2.0-t3.*t8.*wandamax.*(1.3e+1./2.0)+alpha.*t2.*t9.*t12.*t23.*pi.*(ka./2.0+1.0))-W_t.*alpha.*t8.*(1.3e+1./2.0)];