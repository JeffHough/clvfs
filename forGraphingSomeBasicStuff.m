% This is used to get some plots for visualization, where applicable.

    r = -10:0.001:10;
    kc = 1;
    b = 4;
    
    v = zeros(size(r));
    
    for k = 1:numel(v)
        if r(k)>0
            if r(k) > b
                v(k) = -kc;
            else
                v(k) = kc/b^2*(r(k))^2 - 2*kc/b*r(k);
            end
        else
            if -r(k)>b
                v(k) = kc;
            else
                v(k) = -kc/b^2*(r(k))^2 - 2*kc/b*r(k);
            end
        end
    end
    figure;
    pdfprep()
    close(gcf);
    
figure
plot(r,v);
grid on
hold on
ylim([-1.1 1.1])
xlabel('$r-\alpha$ (m)','interpreter','latex');
ylabel('$v_c$ (m/s)','interpreter','latex');
pdfplot2(gcf,'newVc');





