function [aircraftChosen, aircraftName, hTrimFL, iasKTS, ms, km] = utGetTrimPoint(groupe)
    %https://fr.mathworks.com/help/matlab/ref/table.html
    groupName = {'alpha'; 'bravo'; 'charlie'; 'delta'; 'echo'; 'foxtrot'; 'golf'; 'hotel'; 'india'; 'juliette'; 'prof'};
    aircraft = {'Airbus A-320';'Boeing 737-800';'Airbus A-319';'Airbus A-321';'Boeing 737-700';'Boeing 737-300'; 'Airbus A-320';'Boeing 737-800';'Airbus A-319';'Airbus A-321';'Airbus A-321'};
    FL = [130; 150; 200; 250; 300; 130; 150; 200; 250; 290; 300];
    IAS = [260; 260; 270; 280; 290; 260; 260; 270; 280; 290; 270]; %kts
    mStat = [0.2; 0.2; 0.3; 0.3; 0.4; 0.5; 0.5; 0.4; 0.4; 0.3; 0.6]; %marge statique
    kmass = [0.5; 0.2; 0.7; 0.9; 0.8; 0.3; 0.3; 0.5; 0.4; 0.7; 0.3]; %coefficient masse

    trim = table(groupName, aircraft, FL, IAS, mStat, kmass);

    % Recherche de 'groupe' dans la table 'groupName'
    % L'indice trouve dans la table est place dans 'choix'
    groupChosen = strfind(groupName, groupe);    
    for choix=1:size(groupName,1)
        if ~isempty(groupChosen{choix}) 
            break
        end
    end

    aircraftName = trim.aircraft{choix};
    
    % Recherche de 'aircraftName' dans la table 'aircraft'
    k = strfind(aircraft,aircraftName);    
    for aircraftChosen=1:size(aircraft,1)
        if ~isempty(k{aircraftChosen}) %indice dans le tableau
            break
        end
    end

    hTrimFL = trim.FL(choix);
    iasKTS = trim.IAS(choix);
    ms = trim.mStat(choix); %marge statique
    km = trim.kmass(choix); %coefficient de masse =1 pour MTOW


