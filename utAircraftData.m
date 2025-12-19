function [F0, lambda, lambdat, S, St, Lfus, lt, cbar, OWE, MTOW, Cx0, ki] = utAircraftData(aircraftChosen)

	%aircraftType={'Airbus A-320','Boeing 737-800','Airbus A-319','Airbus A-321','Boeing 737-700','Boeing 737-300'};
	iF0 = 1; ilambda=2; ilambdat=3; iS=4; iSt=5; icbar=6; iLfus=7; iMTOW=8; iOWE=9; iCx0=10; iki=11;
	sizeAircraftDataSet = 11;
	
	aircraftData = zeros(sizeAircraftDataSet,1);
	%aircraftData(:,iF0) = [2*111205;2*106757; 2*97860; 2*133446; 2*91633; 2*88694 ];
    
    %!!! Verifier l'ordre dans la table 'aircraft' de la fonction utGetTrimPoint() !!!
	switch uint8(aircraftChosen) %uint8: Convert to unsigned 8-bit integer (pour pouvoir faire le switch)
		case  1 %"Airbus A-320"
			aircraftData(iF0) = 2*111205;
			aircraftData(ilambda:iLfus) = [9.39 , 5 , 122.44 , 31 , 4.19 , 37.57];
			aircraftData(iMTOW:iOWE) = [73500 , 39733];
			aircraftData(iCx0:iki) = [.26659E-01 , .38726E-01];

		case 2 %"Boeing 737-800"
			aircraftData(iF0) = 2*106757;
			aircraftData(ilambda:iLfus) = [9.45 , 6.28  , 124.6 , 32.8 , 4.17 , 38.02];
			aircraftData(iMTOW:iOWE) = [70534 , 41413];
			aircraftData(iCx0:iki) = [.25452E-01 , .35815E-01];

		case 3 %"Airbus A-319"
			aircraftData(iF0) = 2*97860;		
			aircraftData(ilambda:iLfus) = [9.39 , 5 , 122.44 , 31 , 4.19 , 33.84];
			aircraftData(iMTOW:iOWE) = [64000 , 39358];
			aircraftData(iCx0:iki) = [.25954E-01 , .25882E-01];

		case 4 %"Airbus A-321"
			aircraftData(iF0) = 2*133446;		
			aircraftData(ilambda:iLfus) = [9.13 , 5 , 126 , 31 , 4.34 , 44.51];
			aircraftData(iMTOW:iOWE) = [89000 , 47000];
			aircraftData(iCx0:iki) = [.26984E-01 , .35074E-01];

		case 5 %"Boeing 737-700"
			aircraftData(iF0) = 2*91633;		
			aircraftData(ilambda:iLfus) = [9.44 , 6.28 , 124.6 , 32.8 , 4.17 , 32.18];
			aircraftData(iMTOW:iOWE) = [60326 , 37648];
			aircraftData(iCx0:iki) = [.23738E-01 , .37669E-01];

		case 6 %"Boeing 737-300"
			aircraftData(iF0) = 2*88694;		
			aircraftData(ilambda:iLfus) = [9.16 , 5.15 , 91.04 , 31.31, 3.73 , 32.18];
			aircraftData(iMTOW:iOWE) = [56473 , 31480];
			aircraftData(iCx0:iki) = [.24958E-01 ,  .40885E-01];
	end
	
    F0 = aircraftData(iF0); %N
    lambda = aircraftData(ilambda);
    lambdat = aircraftData(ilambdat);
    S = aircraftData(iS); %m^2
    St = aircraftData(iSt); %m^2
    Lfus = aircraftData(iLfus); %m
    lt = 0.5*Lfus; 
    cbar = aircraftData(icbar); %m
    OWE = aircraftData(iOWE); %kg
    MTOW = aircraftData(iMTOW); %kg
    Cx0  = aircraftData(iCx0);
    ki   = aircraftData(iki);
end 
