function [CL,CLw,CLt,CD,Cm]=utCoeffAero(alpha,dPHR,q,Va,lambda,lambdat,S,St,lt,cbar,ms,elevator)
    %ms: marge statique
    Vt = (lt*St)/(cbar*S); %volume d'empennage
    
    [CD0, alpha0, Cm0, CLtq] = utConstantAeroCoeff();
    
    CLw = utCLalpha(lambda)*(alpha - alpha0);
    
    alpha_t = alpha - 0.25*(alpha - alpha0) + dPHR;
    if (Va ~= 0.)
        alpha_t = alpha_t + CLtq*(q*lt/Va);
    end
    CLt = utCLalpha(lambdat)*alpha_t;
    
    CL = CLw + St/S*CLt;
    useStall = true;
    if useStall %post stall modelling
        Nstall = 4.2; 
        Cstall = 2e-6;
        %Model inspired from the following paper:
        %Development of Aerodynamic Modeling and Calibration
        %Methods for General Aviation Aircraft Performance
        %Analysis - a Survey and Comparison of Models
        %Sanggyu Min, Evan Harrison, Hernando Jimenez, Dimitri Mavris
        %AIAA AVIATION Forum
        %22-26 June 2015, Dallas, TX
        %15th AIAA Aviation Technology, Integration, and Operations Conference
        RAD2DEG = 180/pi;
        CL = CL - Cstall*(RAD2DEG*abs(alpha - alpha0))^Nstall;
	end  
      
    useNewCDi = true;
    if useNewCDi %calcul de la trainee induite
        CDi = CLw^2 / (pi*lambda) + St/S*CLt^2 / (pi*lambdat) + St/S*CLw*CLt / (pi*lambda);
    else %nouveau calcul de la trainee induite
        e = 1; %coefficient d'Oswald
        ki = 1/(e*pi*lambda);
        CDi = ki*CL^2;
    end
    
    CD = CD0 + CDi;
    
    Cmdel = -1.2; %coefficient d'efficacite de la gouverne de profondeur
    del = elevator;
    
    Cm = Cm0 - ms*CLw - Vt*utCLalpha(lambdat)*dPHR + Cmdel*del;
    if (Va ~= 0.)
        Cm = Cm -Vt*utCLalpha(lambdat)*CLtq*(q*lt/Va);
    end
end
