function [CD0, alpha0, Cm0, CLtq] = utConstantAeroCoeff()
	DEG2RAD = pi/180.; 
	
    CD0 = 0.025; %0.13;
    alpha0 = -2*DEG2RAD; %-2*DEG2RAD; %rad
    Cm0 = -0.59;
    CLtq = 1.3;
end
