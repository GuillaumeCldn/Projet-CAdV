%-------------------------
function y=utRho(h)
    %masse volumique de l'air sec a l'altitude h
    %h en [m]
    %y en kg/m^3
    rho0 = 1.225; %kg/m^3
    Rs= 287.05; % m^2 / ( K s^2)
    beta_T = -6.5/1000; %K/m  
    gGravite = 9.80665; % m/s^2  
    y = rho0*(utTemp(h)/utTemp(0))^(-(gGravite/(Rs*beta_T))-1);
end
