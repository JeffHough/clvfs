function damaxLVFdp = grad_p_amaxLVF(alphap,fact,theta_d,vmax,wandamax,wmax)
%GRAD_P_AMAXLVF
%    DAMAXLVFDP = GRAD_P_AMAXLVF(ALPHAP,FACT,THETA_D,VMAX,WANDAMAX,WMAX)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    22-Mar-2021 00:45:02

t2 = 1.0./theta_d;
t3 = t2.*pi.*3.62306e-1;
t4 = t3+7.24612e-1;
damaxLVFdp = [wandamax-1.0./alphap.^2.*fact.*t4.*vmax.^2.*pi;wmax.*2.0+(fact.*t4.*vmax.*pi.*2.0)./alphap];
