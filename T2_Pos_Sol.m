function out1 = T2_Pos_Sol(T_man,omega_f,omega_i,theta_f,theta_i)
%T2_POS_SOL
%    OUT1 = T2_POS_SOL(T_MAN,OMEGA_F,OMEGA_I,THETA_F,THETA_I)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    17-Sep-2021 08:37:41

t2 = T_man.*omega_f;
t3 = theta_i.*2.0;
t4 = T_man.*omega_i.*2.0;
t5 = theta_f.*-2.0;
out1 = (t2+t3+t4+t5+sqrt(t2.*t3+t2.*t5-theta_f.*theta_i.*8.0+theta_f.^2.*4.0+theta_i.^2.*4.0+T_man.^2.*omega_i.^2+T_man.*omega_i.*t3+T_man.*omega_i.*t5))./(omega_f+omega_i.*3.0);
