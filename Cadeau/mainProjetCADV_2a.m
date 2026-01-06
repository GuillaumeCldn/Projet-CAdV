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
groupe = 'alpha' %rentrer son groupe
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

%% Modes


Atheta = A(iVa:iq,iVa:iq)
Btheta = B(iVa:iq , idelevator)
Ctheta = [0 ,  0, 1, 0]
[numFtheta,denFtheta] = ss2tf(Atheta, Btheta, Ctheta, 0)
Ftheta = tf(numFtheta, denFtheta)

eig(Atheta)
[Wn,Z] = damp(Atheta)

%specification
ts = 3 %sec
D = 5/100
[Kp, Ki, Kd, m, w0, dp] = utWang(Ftheta, ts, D,-4.7) %on prend kp = -4.7

FTBF_in = feedback(Ftheta , tf([Kd,0],1)) %fonction de transfert 'interne'
FTBF = feedback( tf([Kp,Ki], [1,0])*FTBF_in,  1)
pole(FTBF)
zero(FTBF)

minFTBF = minreal(FTBF, 1e-4)

q0 = minFTBF.den{1}
Cpf = tf(q0(end),minFTBF.num{1})
Cpf1 = tf(1,[1/1.05,1])

%% Comande par retour d'état





%Linearisation
%vecteur d'etat au point de trim
xTrimcm = zeros(8, 1);
xTrimcm(ihp) = hTrim; %m
xTrimcm(iVa) = VaTrim; %m/sec
xTrimcm(ialpha) = alphaTrim; %rad
xTrimcm(itheta) = thetaTrim; %rad
%vecteur de commande au point de trim choisi
uTrimcm = 0

%linearisation autour du point de trim
[Acm, Bcm, Ccm, Dcm] = linmod('ToLinDeltaTheta',xTrimcm, uTrimcm)

%Commandablite du systeme
rank(ctrb(Acm,Bcm))

Kcm = place(Acm,Bcm,[dp,conj(dp),-5,-6,-7,-8,-9,-10])
eig(Acm-Bcm*Kcm)





%Observabilité du systeme
rank(obsv(Acm,Ccm))

A6 = Acm(iVa:end,iVa:end)
B6 = Bcm(iVa:end )

%precommande
H = -inv(Ccm*inv(Acm-Bcm*Kcm)*Bcm)