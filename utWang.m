function [Kp, Ki, Kd, m, w0, dp] = utWang(F, ts, D, kp_bypass)
    
    % Qing-Guo Wang, Zhiping Zhang, Karl Johan Astrom, Yu Zhang, Yong Zhang
    % Guaranteed Dominant Pole Placement with PID Controllers,
    % IFAC Proceedings Volumes,
    % Volume 41, Issue 2,
    % 2008,
    % Pages 5842-5845,
    % ISSN 1474-6670,
    % ISBN 9783902661005,
    % https://doi.org/10.3182/20080706-5-KR-1001.00985.
    % (https://www.sciencedirect.com/science/article/pii/S1474667016398779)
    % Keywords: PID Controllers; Dominant Poles; Pole Placement; 
    % Root-Locus; Nyquist Plot

    % Calcul du coefficient d'amortissmeent et de la pulsation propre Ã 
    % partir des exigences

    m = -log(D)/sqrt(pi^2 + log(D)^2);
    w0 = 4/(m*ts);

    % % Figure echellon du systeme definit par les exigences
    % figure
    % set(gcf,'Visible','on')
    % step(tf(w0^2, [1 2*m*w0 w0^2]))
    % title("RÃ©ponse Ã  un Ã©chelon en fonction des spÃ©cifications")

    % figure
    % set(gcf,'Visible','on')
    % plot([-m*w0 -m*w0],[-w0*sqrt(1-m^2) w0*sqrt(1-m^2)], 'xr') 
    % hold on
    % rlocus(-F, 'b')
    % legend("PÃ´les dominants souhaitÃ©s", "SystÃ¨me bouclÃ© par un gain unitaire nÃ©gatif")

    % Amortissement 
    dp = -m*w0+1i*w0*sqrt(1-m^2); 

    a = real(dp);
    b = imag(dp);
    Fdp = polyval(F.num{1},dp)/polyval(F.den{1},dp);

    X1 = (1/(2*b))*imag(-1/Fdp)-(1/(2*a))*real(-1/Fdp);
    X2 = (1/(2*b))*imag(-1/Fdp)+(1/(2*a))*real(-1/Fdp);
    
    if ~exist('kp_bypass','var')

        L = tf(conv(-F.num{1},[1 -2*a (a^2+b^2)]),conv(2*a*F.den{1},[0, 1, 0]) + conv(F.num{1},[2*a*X2, 0, -2*a*(a^2+b^2)*X1]));
        figure
        set(gcf,'Visible','on')
        subplot(121);
        rlocus(-L)
        hold on
        plot([-m*w0 -m*w0],[-w0*sqrt(1-m^2) w0*sqrt(1-m^2)], 'xr') 
        title('-L')
        legend("Pôles dominants souhaités", "Système bouclé par un gain unitaire négatif")
        subplot(122);
        rlocus(L)
        hold on
        plot([-m*w0 -m*w0],[-w0*sqrt(1-m^2) w0*sqrt(1-m^2)], 'xr') 
        title('L')
        legend("Pôles dominants souhaités", "Système bouclé par un gain unitaire positif")
        
        prompt = {'Valeur de Kp'};
        dlgtitle = 'Valeur de Kp';
        opts.WindowStyle = 'normal';
        answer  = inputdlg(prompt,dlgtitle, 1, {' '}, opts);
    
        Kp =  str2num(answer{1});
    else
        Kp = kp_bypass;
    end

    Ki = -(a^2+b^2)*((Kp/(2*a))+X1);
    Kd = -(Kp/(2*a)) + X2;
end