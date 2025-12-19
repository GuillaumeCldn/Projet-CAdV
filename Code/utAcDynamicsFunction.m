function xdot = utAcDynamicsFunction(x,u,aircraftChosen,kmass,ms)
	%-------------------------
	%Conversion des donnees
	KTS2MS = 1852 / 3600;
	FL2M = 30.48 ;
	RAD2DEG = 180/pi; 
	DEG2RAD = pi/180.; 
	FPM2MS = 0.3048/60;

	gGravite = 9.80665; % m/s^2

	%-------------------------
	% Indexation des composantes du vecteur d'etat
	iy = 1; % m
	ihp = 2; % m
	iVa = 3;  % m/s
	ialpha = 4; % rad
	itheta = 5; % rad
	iq = 6; % rad/sec
	TOTAL_SV = 6;

	%-------------------------
	% Indexation des composantes du vecteur de commande
	idPHR = 1;
	idthr = 2;
	idelevator = 3;
	TOTAL_CMD = 3;

	%-------------------------
	%utThrust
	%utConstantAeroCoeff
	%utCLalpha
	%utCoeffAero
	%utAeroForcesAndMoments
	%utAcMass
	%utComputeIyy
	%-------------------------

	%Wy, Wh: vitesse du vent, m/s
    
    Wy = 0; %vitesse horizontale du vent, m/s
	Wh = 0; %vitesse verticale du vent, m/s
  
  	[F0, lambda, lambdat, S, St, Lfus, lt, cbar, OWE, MTOW, Cx0, ki] = utAircraftData(aircraftChosen);
    mass = utAcMass(kmass, OWE, MTOW);
    Iyy = utComputeIyy(mass,Lfus);
    
	hp = x(ihp); %m
    Va = x(iVa); %m/s
    alpha = x(ialpha); %rad
    theta = x(itheta);    
    q = x(iq); %rad/sec
    dthr = u(idthr); %sans unite
    dPHR = u(idPHR); %rad
    delevator = u(idelevator); %rad
    
	[F,L,D,M] = utAeroForcesAndMoments(Va, hp, alpha, q, dthr, dPHR, delevator, F0,lambda,S,lambdat,St,lt,cbar,ms);
            
    xdot = zeros(TOTAL_SV,1);
    xdot(iy) = Va*cos(theta-alpha) + Wy;
    xdot(ihp) = Va*sin(theta-alpha) + Wh;
    xdot(iVa) = (F*cos(alpha)-D)/mass - gGravite*sin(theta-alpha);
    xdot(ialpha) = x(iq) - (L + F*sin(alpha))/(mass*Va) + gGravite/Va*cos(theta-alpha);
    xdot(itheta) = x(iq);
    xdot(iq) = M/Iyy;
end

