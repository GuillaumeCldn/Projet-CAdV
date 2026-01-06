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
groupe = 'golf' %rentrer son groupe
[aircraftChosen, aircraftName, hTrimFL, Eas_KTS, ms, km] = utGetTrimPoint(groupe);

VaTrim = utEas2Tas(Eas_KTS*KTS2MS, hTrimFL*FL2M);
hTrim = hTrimFL*FL2M;
dthrIdle = 0.1;
trimVal = utComputeTrimIdle(hTrim,VaTrim,aircraftChosen,km,ms,dthrIdle); %trim thrust idle
dPHRTrim = trimVal(1); %rad
alphaTrim = trimVal(2); %rad
thetaTrim = trimVal(3); %rad
fpaDeg = RAD2DEG*(thetaTrim-alphaTrim); %pente, flight path angle

%Linearisation
%vecteur d'etat au point de trim
xTrim = zeros(TOTAL_SV, 1);
xTrim(ihp) = hTrim; %m
xTrim(iVa) = VaTrim; %m/sec
xTrim(ialpha) = alphaTrim; %rad
xTrim(itheta) = thetaTrim; %rad
% Vecteur de commande au point de trim choisi
uTrim = zeros(TOTAL_CMD, 1);
uTrim(idPHR) = dPHRTrim;
uTrim(ithr) = dthrIdle;
uTrim(idelevator) = 0;

% Vérification trim
xdotTrim = utAcDynamicsFunction(xTrim,uTrim,aircraftChosen,km,ms);

% Linéarisation autour du point de trim
[A, B, C, D] = linmod('acDynModel_ToLinearize_2015',xTrim, uTrim);


A4 = A(iVa:iq, iVa:iq);
B4 = B(iVa:iq, idelevator);

C4 = [0, 0, 1, 0];
D4 = 0;

[state4num, state4den] = ss2tf(A4, B4, C4, D4);
state4 = tf(state4num, state4den);

% Calcul des modes de A4
modes4 = eig(A4);
[Wn, zeta] = damp(A4);
 % Nom des modes ? Deux pôles normaux et deux pôles très lents, principalement oscillatoires ? 

% Specifications 
ts = 3; % secondes
D = 0.05; % %
[Kp, Ki, Kd, m, w0, dp] = utWang(state4, ts, D, -5.66); % On prend Kp = - 5.66

% Préfiltre
FTBF_in = feedback(state4, tf([Kd, 0], 1)); % Fonction de transfert 'interne'
FTBF = feedback(tf([Kd, Ki], [1, 0])*FTBF_in, 1);
pole(FTBF);
zero(FTBF);

minFTBF = minreal(FTBF, 1e-4);

q = minFTBF.den{1}; % Dénominateur de la FTBF
Cpf = tf(q(end), minFTBF.num{1});
Cpf1 = tf(1,[dp,1]);


%% Commande par retour d'état

% Linearisation

% Vecteur d'etat au point de trim
xTrimcm = zeros(8, 1);
xTrimcm(ihp) = hTrim; %m
xTrimcm(iVa) = VaTrim; %m/sec
xTrimcm(ialpha) = alphaTrim; %rad
xTrimcm(itheta) = thetaTrim; %rad

% Vecteur de commande au point de trim choisi
uTrimcm = 0;

% Linearisation autour du point de trim
[Acm, Bcm, Ccm, Dcm] = linmod('acDynModelGolfTenueClassique',xTrimcm, uTrimcm);

% Commandablite du systeme
rank(ctrb(Acm,Bcm))

% Observabilité du systeme
rank(obsv(Acm,Ccm))

A6 = Acm(iVa:end, iVa:end);
B6 = Bcm(iVa:end);
C6 = Ccm(iVa:end, iVa:end);
pA6 = eig(A6)
K6 = place(A6,B6,[dp, conj(dp), -3.3965 + 0.1541i, -3.3965 - 0.1541i, -5, -6])

% % Precommande
% H = -inv(Ccm*inv(A6-B6*K6)*B6);













