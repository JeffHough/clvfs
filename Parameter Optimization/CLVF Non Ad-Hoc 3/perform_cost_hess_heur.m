function d2Cdp2_heur = perform_cost_hess_heur(W,alpha,b,ka,kc,r,v,wandamax,wmax)
%PERFORM_COST_HESS_HEUR
%    D2CDP2_HEUR = PERFORM_COST_HESS_HEUR(W_F,W_T,ALPHA,B,KA,KC,RX,RY,RZ,VX,VY,VZ,WANDAMAX,WMAX)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    19-Sep-2021 18:09:41

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
t5 = b+2.0;
t6 = b.^2;
t7 = ka.^2;
t8 = rx.^2;
t9 = ry.^2;
t10 = rz.^2;
t11 = wmax.^2;
t12 = 1.0./alpha;
t13 = 1.0./ka.^3;
t14 = 1.0./kc;
t19 = ka./2.0;
t4 = t3.^2;
t15 = t14.^2;
t16 = t14.^3;
t18 = t14.^5;
t20 = t19+1.0;
t21 = t7./4.0;
t24 = t8+t9+t10;
t17 = t15.^2;
t22 = W_t.*t5.*t15;
t23 = t20.^2;
t26 = ka.*t12.*t15.*pi.*4.0;
t27 = 1.0./t24;
t28 = ka+t2+t21+1.0./4.0;
t29 = 1.0./sqrt(t24);
t30 = W_f.*b.*t11.*t12.*t15.*pi.*4.0;
t25 = -t22;
t31 = t28.^2;
t32 = t28.^3;
t33 = -t30;
t34 = kc.*rx.*t29;
t35 = kc.*ry.*t29;
t36 = kc.*rz.*t29;
t37 = t34+vx;
t38 = t35+vy;
t39 = t36+vz;
t43 = t3.*t15.*t31;
t44 = t25+t33;
t40 = t37.^2;
t41 = t38.^2;
t42 = t39.^2;
t45 = t3+t43;
t46 = 1.0./sqrt(t45);
t49 = t40+t41+t42;
t47 = t46.^3;
t48 = alpha.*t2.*t16.*t20.*t28.*t46.*pi.*2.0;
t50 = t2.*1.0./t12.^3.*t18.*t20.*t32.*t47.*pi;
t51 = -t50;
t52 = t26+t48+t51;
t53 = W_f.*t52;
t54 = -t53;
d2Cdp2_heur = reshape([W_t.*t14+W_f.*t11.*t12.*t14.*pi.*4.0,t44,0.0,t44,W_f.*(1.0./t49.^(3.0./2.0).*(rx.*t29.*t37.*2.0+ry.*t29.*t38.*2.0+rz.*t29.*t39.*2.0).^2.*(-1.0./4.0)+(1.0./sqrt(t49).*(t8.*t27.*2.0+t9.*t27.*2.0+t10.*t27.*2.0))./2.0+t12.*t16.*pi.*(t7+t6.*t11).*4.0-t2.*1.0./t12.^3.*t15.^3.*t31.^2.*t47.*pi+alpha.*t2.*t17.*t31.*t46.*pi.*3.0)+W_t.*((t16.*2.0)./t29+t16.*(b.*2.0+t6./2.0+1.0./2.0).*2.0),t54,0.0,t54,W_f.*(t12.*t14.*pi.*4.0+t3.*t13.*wandamax.*1.3e+1+alpha.*t2.*t15.*t23.*t46.*pi+(alpha.*t2.*t15.*t28.*t46.*pi)./2.0-t2.*1.0./t12.^3.*t17.*t23.*t31.*t47.*pi)+W_t.*alpha.*t13.*1.3e+1],[3,3]);
