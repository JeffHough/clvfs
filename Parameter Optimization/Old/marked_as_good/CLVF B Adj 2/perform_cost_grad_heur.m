function dCdp_heur = perform_cost_grad_heur(W,alpha,b,ka,kc,r,v,wandamax,wmax)
%PERFORM_COST_GRAD_HEUR
%    DCDP_HEUR = PERFORM_COST_GRAD_HEUR(W_F,W_T,ALPHA,B,KA,KC,RX,RY,RZ,VX,VY,VZ,WANDAMAX,WMAX)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    13-Sep-2021 23:32:59

W_t = W(1);
W_f = W(2);

rx = r(1);
ry = r(2);
rz = r(3);

vx = v(1);
vy = v(2);
vz = v(3);

t2 = alpha.*wmax;
t3 = alpha.^2;
t4 = b.^2;
t5 = ka.^2;
t6 = rx.^2;
t7 = ry.^2;
t8 = rz.^2;
t9 = 1.0./alpha;
t11 = 1.0./kc;
t10 = 1.0./t5;
t12 = t11.^2;
t13 = t5./4.0;
t14 = t6+t7+t8;
t15 = ka+t2+t13+1.0./4.0;
t16 = 1.0./sqrt(t14);
t17 = t15.^2;
t18 = kc.*rx.*t16;
t19 = kc.*ry.*t16;
t20 = kc.*rz.*t16;
t21 = t18+vx;
t22 = t19+vy;
t23 = t20+vz;
t24 = t3.*t12.*t17;
t25 = t3+t24;
t26 = 1.0./sqrt(t25);
dCdp_heur = [W_t.*t11.*(b+2.0)+W_f.*b.*t9.*t11.*wmax.*2.0;-W_t.*(t12./t16+t12.*(b.*2.0+t4./2.0+1.0./2.0))+W_f.*(((rx.*t16.*t21.*2.0+ry.*t16.*t22.*2.0+rz.*t16.*t23.*2.0).*1.0./sqrt(t21.^2+t22.^2+t23.^2))./2.0-t4.*t9.*t12.*wmax-alpha.*t2.*t11.^3.*t17.*t26.*pi.*2.0+1.0);W_f.*(pi-t3.*t10.*wandamax.*1.3e+1+alpha.*t2.*t12.*t15.*t26.*pi.*(ka./2.0+1.0).*2.0)-W_t.*alpha.*t10.*(1.3e+1./2.0)];
