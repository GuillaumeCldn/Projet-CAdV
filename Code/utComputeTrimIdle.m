function [trimVal,fval,exitFlag] = utComputeTrimIdle(hTrim,VaTrim,aircraftChosen,kmass,ms,dthrIdle)
	iy = 1; % m
	ihp = 2; % m
	iVa = 3;  % m/s
	ialpha = 4; % rad
	itheta = 5; % rad
	iq = 6; % rad/sec
	TOTAL_SV = 6;
    
	function xdotTrim = setTrim(u)
		dPHR = u(1);
		x = zeros(TOTAL_SV,1); %state vector
		x(ihp) = hTrim; % m
		x(ialpha) = u(2); % rad
		x(itheta) = u(3); % rad
		x(iVa) = VaTrim; % m/s
		delevator = 0;
		xdot = utAcDynamicsFunction(x,[dPHR;dthrIdle;delevator], aircraftChosen,kmass,ms);
              
        xdotTrim = xdot(iVa:iq);
    end

	function xdotTrim = setTrimFzero(u)
        xdotTrim = norm(setTrim(u));
    end

    dPHRTrim0 = 0; %rad
    alphaTrim0 = 0; %rad
    thetaTrim0 = 0; %rad
    if true
        options = optimoptions('fsolve','Algorithm','levenberg-marquardt');
        [trimVal,fval,exitFlag] = fsolve(@setTrim,[dPHRTrim0;alphaTrim0;thetaTrim0],options);
    else
        [trimVal,fval,exitFlag] = fminsearch(@setTrimFzero, [dPHRTrim0;alphaTrim0;thetaTrim0]);
    end
end
