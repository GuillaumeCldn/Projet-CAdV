function [F,L,D,M] = utAeroForcesAndMoments(Va, hp, alpha, q, dthr, dPHR, delevator, F0,lambda,S,lambdat,St,lt,cbar,ms)
    %Va = x(iVa); %m/s
    %hp = x(ihp); %m
    %alpha = x(ialpha); %rad
    %q = x(iq); %rad/sec
    %dthr = u(idthr); %sans unite
    %dPHR = u(idPHR); %rad
    %delevator = u(idelevator); %rad
    qbar = 0.5*utRho(hp)*Va^2; %pression dynamique
    %poussee moteur
    F = utThrust(utTas2Mach(Va,hp),hp,dthr,F0);
    
    [CL,CLwb,CLt,CD,Cm]=utCoeffAero(alpha,dPHR,q,Va,lambda,lambdat,S,St,lt,cbar,ms,delevator);
    
    %portance
    L = qbar*S*CL; %qbar*(S*CLwb + St*CLt);
    
    %trainee
    D = qbar*S*CD;
    
    %moment de tangage
    M = qbar*S*cbar*Cm;
end 
