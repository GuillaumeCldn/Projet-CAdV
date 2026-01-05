clear all
close all
%bdclose all %Close any or all Simulink systems
clc

%Conversion des unités
KTS2MS = 1852 / 3600;
FL2M = 30.48 ;
RAD2DEG = 180/pi; 
DEG2RAD = pi/180.; 
FPM2MS = 0.3048/60;
gGravite = 9.80665; % m/s^2

%Indexation des composantes du vecteur d'etat
iy = 1; % m
ihp = 2; % m
iVa = 3;  % m/s
ialpha = 4; % rad
itheta = 5; % rad
iq = 6; % rad/sec
TOTAL_SV = 6;

%Indexation des composantes du vecteur de commande
idPHR = 1;
ithr = 2;
idelevator = 3;
TOTAL_CMD = 3;

%Choix de l'équipe
groupe = 'prof' %rentrer son groupe
[aircraftChosen, aircraftName, hTrimFL, Eas_KTS, ms, km] = utGetTrimPoint(groupe);

VaTrim = utEas2Tas(Eas_KTS*KTS2MS, hTrimFL*FL2M);
hTrim = hTrimFL*FL2M;
dthrIdle = 0.1;
trimVal = utComputeTrimIdle(hTrim,VaTrim,aircraftChosen,km,ms,dthrIdle); %trim thrust idle
dPHRTrim = trimVal(1) %rad
alphaTrim = trimVal(2) %rad
thetaTrim = trimVal(3) %rad
fpaDeg = RAD2DEG*(thetaTrim-alphaTrim) %pente, flight path angle

%Linearisation
%vecteur d'etat au point de trim
xTrim = zeros(TOTAL_SV, 1);
xTrim(ihp) = hTrim; %m
xTrim(iVa) = VaTrim; %m/sec
xTrim(ialpha) = alphaTrim; %rad
xTrim(itheta) = thetaTrim; %rad
%vecteur de commande au point de trim choisi
uTrim = zeros(TOTAL_CMD, 1);
uTrim(idPHR) = dPHRTrim;
uTrim(ithr) = dthrIdle;
uTrim(idelevator) = 0

%verification trim
xdotTrim = utAcDynamicsFunction(xTrim,uTrim,aircraftChosen,km,ms)

%linearisation autour du point de trim
[A, B, C, D] = linmod('acDynModel_ToLinearize_2015',xTrim, uTrim)
