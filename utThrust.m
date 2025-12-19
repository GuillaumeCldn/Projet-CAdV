function y=utThrust(Ma,h,dthr,F0)
    %Va: vitesse aerodynamique en m/s
    %h: altitude pression en m
    %dthr: position de la manette des gaz (0 <= dthr <= 1)
    %F0: poussée maximale au sol, N
    
    %y = dthr*F0*((utRho(h)/utRho(0))^0.6)*(0.568+0.25*(1.2-Ma)^3); %Mattingly
    l = 0.6; %l=0.6 => Mattingly
    y = dthr*F0*((utRho(h)/utRho(0))^l)*(0.568+0.25*(1.2-Ma)^3);
end
