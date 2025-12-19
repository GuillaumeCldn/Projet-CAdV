function EasDot = utGetEasDot(h, Va, Vadot, alpha, theta)
    %h en [m]
    %alpha et theta en [rad]
    %Va: vitesse aerodynamique en [m/s]
    %Vadot en [m/s^2]
    rho0 = 1.225; %kg/m^3
    Rs= 287.05; % m^2 / ( K s^2)
    beta_T = -6.5/1000; %K/m  
    gGravite = 9.80665; % m/s^2  
    
    drho_dh = -rho0*((gGravite/(Rs*beta_T))+1)*beta_T/utTemp(0)*(utTemp(h)/utTemp(0))^(-(gGravite/(Rs*beta_T))-2);        
    EasDot = drho_dh/(2*sqrt(utRho(0)*utRho(h)))*sin(theta-alpha)*Va^2 + sqrt(utRho(h)/utRho(0))*Vadot;
        
end

