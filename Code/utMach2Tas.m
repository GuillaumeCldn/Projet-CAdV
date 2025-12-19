function y=utMach2Tas(Ma,h)
    %Va: vitesse aerodynamique en m/s
    %h: altitude pression en m
    kapa=1.4;
    Rs= 287.05; % m^2 / ( K s^2)
    y = Ma.*(kapa*Rs*utTemp(h))^0.5;
end
