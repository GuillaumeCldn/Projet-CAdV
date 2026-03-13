# Projet Commande Automatique du Vol

**Groupe :** GOLF  
**Formation :** IENAC24-AVI (Ingénieur ENAC, majeure AVIonique)  

Ce projet a pour but de modéliser la dynamique du vol d'un avion et d'implémenter des lois de commande spécifiques de vol automatique.

## Objectif du projet

L'objectif principal de ce projet est d'implémenter une loi de commande **OPEN Descent** pour un avion de ligne (modèle Airbus A320 pour le groupe GOLF). 

Cela implique :
- La modélisation de la dynamique longitudinale de l'avion (équations de forces, de moments, calcul de la poussée, masse, et paramètres aérodynamiques).
- Le calcul du point d'équilibre (Trim).
- Le contrôle de la descente et du palier en commandant l'élévateur, le trim du plan horizontal réglable (PHR) et la poussée des moteurs. 

## Commandes utilisées

Nous avons développé trois commandes différentes pour ce projet:

- pour la descente:
    - une commande classique (PID) pour le contrôle de l'assiette
    - une commande modale SISO pour le contrôle de la vitesse
- pour le palier:
    - une commande modale MIMO pour contrôler l'assiette et la vitesse

## Structure du dépôt

Le dépôt est organisé de la manière suivante :

* **`Code/`** : Contient l'ensemble des scripts MATLAB (`.m`) utilisés pour la simulation et la commande de l'avion.
    *Les fonctions utilitaires suivantes ont été fournies par nos professeurs:*
  * **Fonctions de dynamique et d'aérodynamique :** `utAcDynamicsFunction.m`, `utAeroForcesAndMoments.m`, `utCLalpha.m`, `utThrust.m`...
  * **Fonctions de conversion et d'atmosphère :** `utTas2Eas.m`, `utRho.m`, `utTemp.m`...
  * **Gestion des données avion et trim :** `utAircraftData.m` (données de plusieurs modèles d'avions commerciaux), `utGetTrimPoint.m` (initialisation des conditions de vol par groupe), `utComputeTrimIdle.m`.
* **`Rapport/`** : Dossier contenant le rapport final du projet et ses ressources.
* **`Sujet CaDV.pdf`** : Le sujet officiel détaillant les exigences du projet.
* **`consignes_rapport_projet_IENAC24_Avi.pdf`** : Les consignes de rédaction pour le rapport IENAC.
* **`Golf_Matlab.zip`** : Archive contenant les codes du groupe GOLF.
* **`LICENSE`** : La licence associée à ce dépôt.

##  Résultats et simulation

À l'aide des scripts MATLAB mis à disposition et complétés, le système est capable de :
1. Déterminer les caractéristiques de vol et les données structurelles du modèle d'avion sélectionné.
2. Calculer le point d'équilibre.
3. Simuler la réponse de l'avion à des variations de consigne (poussée, gouverne de profondeur, et plan d'empennage horizontal réglable - PHR) pour respecter une consigne de vol *OPEN Descent*.


Pour consulter les résultats de nos simulations et nos conclusions sur ce projet, lire le rapport: `Rapport/RapportCAdV.pdf`
