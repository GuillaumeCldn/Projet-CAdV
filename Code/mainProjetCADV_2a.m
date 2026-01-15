%% initiation
close all
%bdclose all %Close any or all Simulink systems
clc

%Conversion des unités
KTS2MS = 1852 / 3600;
FL2M = 30.48;
RAD2DEG = 180/pi;
DEG2RAD = pi/180.;
FPM2MS = 0.3048/60;
gGravite = 9.80665; % m/s^2

%Indexation des composantes du vecteur d'etat
iy = 1; % m
ihp = 2; % m
iVa = 3; % m/s
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

%% Commande Classique (de l'assiette)

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


A4 = A(iVa:iq, iVa:iq); % on tronque afin de simplifier le système (on ne veut pas l'altitude ou position)
B4 = B(iVa:iq, idelevator);

C4 = [0, 0, 1, 0];
D4 = 0;

[state4num, state4den] = ss2tf(A4, B4, C4, D4);
state4 = tf(state4num, state4den); % on obtient une fonction de transfert SISO pour controler l'assiette

% Vérification de la controlabilité et observabilité du système
ctrPID = rank(ctrb(A4,B4));
obsPID = rank(obsv(A4,C4));
nPID = size(A4,1);

if ctrPID < nPID, disp('PID non contrôlable'); end
if obsPID < nPID, disp('PID non observable'); end


% Calcul des modes de A4
modes4 = eig(A4);
[Wn, zeta] = damp(A4);

% Specifications 
ts = 3; % secondes
D = 0.05; % dépassement de 5% toléré
[Kp, Ki, Kd, m, w0, dp] = utWang(state4, ts, D, -5.66); % On prend Kp = - 5.66, par lecture du root locus

% Préfiltre
FTBF_in = feedback(state4, tf([Kp, 0], 1)); % Fonction de transfert 'interne' pour assiette
FTBF = feedback(tf([Kp, Ki], [1, 0])*FTBF_in, 1);
pole(FTBF);
zero(FTBF);

minFTBF = minreal(FTBF, 1e-4); %nous avons un pole à presque zero qu'il faut enlever

q = minFTBF.den{1}; % Dénominateur de la FTBF
Cpf = tf(q(end), minFTBF.num{1}); % préfiltre d'ordre 2
Cpf1 = tf(1,[dp,1]); % préfiltre d'ordre 1


%% Commande Modale (de la vitesse pilotée via l'assiette)

% Linearisation

% Vecteur d'etat au point de trim pour commande modale
xTrimcm = zeros(8, 1);
xTrimcm(ihp) = hTrim; %m
xTrimcm(iVa) = VaTrim; %m/sec
xTrimcm(ialpha) = alphaTrim; %rad
xTrimcm(itheta) = thetaTrim; %rad

% Vecteur de commande au point de trim choisi
uTrimcm = 0;

% Linearisation autour du point de trim
[Acm, Bcm, Ccm, Dcm] = linmod('Golf_Nonlin_Classique',xTrimcm, uTrimcm);

% Commandablite et Observabilité du systeme
ctrClassique = rank(ctrb(Acm,Bcm));
obsClassique = rank(obsv(Acm,Ccm));
nClassique = size(Acm,1);
if ctrClassique < nClassique, disp('Système non contrôlable'); end
if obsClassique < nClassique, disp('Système non observable'); end


A6 = Acm(iVa:end, iVa:end); % on tronque pour simplifier le système
B6 = Bcm(iVa:end); 
C6 = Ccm(iVa:end, iVa:end);
C6_pour_h = Ccm(iVa, iVa:end); % on veut stabiliser la vitesse
pA6 = eig(A6); % on trouve des poles à presque zero qu'il faut changer
K6 = place(A6,B6,[dp, conj(dp), -3.3965 + 0.1541i, -3.3965 - 0.1541i, -5, -6]); % On a observé les pôles (-3.3965 + 0.1541i, -3.3965 - 0.1541i) dans eig
% On se contente de les conserver ici, pour soulager les actionneurs. On a
% les poles désirés et aussi des poles rapides.
% Inutile de lutter contre les poles de eig

% On doit identifier quelle ligne de C6 correspond à Va pour l'intégrateur
C_Va = zeros(1, size(A6, 2));
C_Va(1) = 1; % On extrait Va

% Création du système augmenté (Ajout de l'état intégrateur)
A_aug = [A6, zeros(size(A6,1), 1); -C_Va, 0];

B_aug = [B6; 0];

% Calcul des gains sur le système augmenté (taille 7)
% On ajoute un pôle pour l'intégrateur
poles_desires = [dp, conj(dp), -3.3965 + 0.1541i, -3.3965 - 0.1541i, -3, -4, -10.33];

K_aug = place(A_aug, B_aug, poles_desires);

% Séparation des gains pour Simulink
K_x = K_aug(1:end-1); % Gains pour le retour d'état classique
K_i = K_aug(end);% Gain pour l'intégrateur


% Precommande
H = -inv(C6_pour_h*inv(A6-B6*K6)*B6);

% Estimateur
Cobs = [C6(1,:); C6(3,:); C6(4,:)]; %on veut se servir de la vitesse, de l'assiette, et le tangage

%Vérification de l'observabilité et controlabilité de l'estimateur
obsLuenberger = rank(obsv(A6,Cobs));
ctrLuenberger = rank(ctrb(A6, B6));
nLuenberger = size(A6,1);
if ctrLuenberger < nLuenberger, disp('Estimateur Luenberger non contrôlable'); end
if obsLuenberger < nLuenberger, disp('Estimateur Luenberger non observable'); end


L6t = place(A6', Cobs', poles_desires(1:6));
L6 = L6t';
eig(A6-L6*Cobs);

%% Commande Optimale/MIMO

taero = 20; % Temps de réponse capture vitesse aérodynamique
Daero = 0; % Dépassement à la réponse indicielle

tpente = 2; % Temps de réponse capture pente
Dpente = 0; % Dépassement à la réponse indicielle

%linéarisation en palier
hTrimLF = 30*FL2M; % m
VaTrimLF = utEas2Tas(230*KTS2MS, hTrimLF); % m/s, 230 depuis la consigne
trimValLF = utComputeTrimLevelFlight(hTrimLF,VaTrimLF,aircraftChosen,km,ms); %trim thrust idle (dphr, alpha, dthr)

dPHRTrimLF = trimValLF(1); %rad
alphaTrimLF = trimValLF(2); %rad
thetaTrimLF = trimValLF(2); %rad
dthrTrimLF = trimValLF(3);
fpaDegLF = RAD2DEG*(thetaTrimLF-alphaTrimLF); %pente, flight path angle

xTrimLF = zeros(TOTAL_SV, 1);
xTrimLF(ihp) = hTrimLF; %m
xTrimLF(iVa) = VaTrimLF; %m/s
xTrimLF(ialpha) = alphaTrimLF; %rad
xTrimLF(itheta) = thetaTrimLF; %rad

% Vecteur de commande au point de trim choisi
uTrimLF = zeros(TOTAL_CMD, 1);
uTrimLF(idPHR) = dPHRTrimLF;
uTrimLF(ithr) = trimValLF(3);
uTrimLF(idelevator) = 0;

% Vérification trim
xdotTrimLF = utAcDynamicsFunction(xTrimLF,uTrimLF,aircraftChosen,km,ms);
[Alf, Blf, Clf, Dlf] = linmod('acDynModel_ToLinearize_2015',xTrimLF, uTrimLF);

% Vérification de controlabilité et observabilité du système linéarisé
ctrMIMO = rank(ctrb(Alf,Blf));
obsMIMO = rank(obsv(Alf,Clf));
nMIMO = size(Alf);
if ctrMIMO < nMIMO, disp('Système MIMO non contrôlable'); end
if obsMIMO < nMIMO, disp('Système MIMO non observable'); end

% Extraction pour LQT
idx_states = iVa:iq; 
idx_inputs = ithr:idelevator; % Entrées : Poussée et Gouverne

A_sys = Alf(idx_states, idx_states);
B_sys = Blf(idx_states, idx_inputs);


C_sys = [1,  0, 0, 0; 
         0, -1, 1, 0];
     
D_sys = zeros(4,2);

% Vérification des pôles du système (pratiquement les mêmes qu'avant)
poles_sys = eig(A_sys);

% Modèle de Référence

Ar = [-3/taero, 0; 
       0,      -3/tpente];

Br = [3/taero, 0; 
      0,       3/tpente];
  
Cr = eye(2);

Dr = zeros(2,2);


Aaug = [A_sys, zeros(4,2);
        zeros(2,4), Ar];
    
Baug = [B_sys; zeros(2,2)];

Caug = [C_sys, -Cr];

Daug = zeros(2,2);

N = [Aaug, Baug;
    Caug, zeros(2,2)];

% Dimensions
n_states = size(A_sys, 1); % 4
n_ref = size(Ar, 1);       % 2
n_inputs = size(B_sys, 2); % 2
n_outputs = size(C_sys, 1);% 2

% Suite Commande Optimale/MIMO : Calcul des gains LQT

% Définition des poids pour le critère quadratique J
% Q : Pénalise l'erreur de suivi e = y - yr
q_Va = 1000; %1 / (40)^2;      % Erreur en vitesse % 40 pour le linéaire marche bien
q_gamma = 1000; %1 / (0.01)^2; % On veut tolérer 0.01 rad (environ 0.5 deg) d'erreur
Q = diag([q_Va, q_gamma]);

% R : Pénalise l'effort de commande u
% u1 = poussée (ratio 0-1), u2 = elevator (rad)
r_th = 1;% / (0.5)^2;     % Pour la poussée % 0.5 en linéaire marche bien
r_el = 1;% / (0.26)^2;     % Pour la gouverne (environ 15 deg) % 0.26 en linéaire marche bien
R = diag([r_th, r_el]);

% Equation de Riccati (LQR augmenté)
% Le critère est e'*Q*e + u'*R*u.
% Comme e = Caug * x_aug, on a x_aug' * (Caug'*Q*Caug) * x_aug
Q_aug = Caug' * Q * Caug;

% Calcul du gain optimal K_aug = [Kx, Kr]
[K_aug, S, E] = lqr(Aaug, Baug, Q_aug, R);

% Extraction des gains
Kx = K_aug(:, 1:4); % Gain de retour d'état (sur Va, alpha, theta, q)
Kr = K_aug(:, 5:6); % Gain sur l'état du modèle de référence

%Vérification de l'observabilité et commandabilité du préfiltre optimal
ctrOPT = rank(ctrb(Ar,Br));
obsOPT = rank(obsv(Ar,-Kr));
nOPT = size(Ar,1);
if ctrOPT < nOPT, disp('Préfiltre optimal non contrôlable'); end
if obsOPT < nOPT, disp('Préfiltre optimal non observable'); end

% Vecteur second membre pour r_ss (Eq 3.71 modifiée)
% Le système est : N * Z = [0; -Br; 0] * r_ss
RHS_Mat = [zeros(n_states, n_ref); 
           -Br; 
           zeros(n_outputs, n_ref)];

% Résolution pour trouver les matrices de gains statiques
% M = [Nx; Nxr; Nu] tel que Z_ss = M * r_ss
M = N \ RHS_Mat;

Nx = M(1:n_states, :);             % Relation x_ss / r_ss
Nxr = M(n_states+1:n_states+n_ref, :); % Relation xr_ss / r_ss
Nu = M(end-n_inputs+1:end, :);     % Relation u_ss / r_ss

% Calcul du gain de pré-commande Dpf (Eq 3.87)
% upf = -Kr*xr + Dpf*r_ss
% u = -Kx*x + upf
% Dpf assure que u_ss est cohérent avec la référence
Dpf = K_aug * [Nx; Nxr] + Nu;















