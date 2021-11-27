function outterAccelerationBound = a_max_CLVF3D(params, rotStuff)

    ka = params(1);
    kc = params(2);
    b = params(3);
    A = params(4);
    
    rotNorm = rotStuff(1);
    w_max = rotStuff(2);

    num = 2/(3*sqrt(3));
    
    outterAccelerationBound = sqrt((kc^2/b + ka^2/A)^2 + (ka^2/(2*A) + ka*w_max*num)^2)  + ka*w_max*(2 + num) + rotNorm*A;

end