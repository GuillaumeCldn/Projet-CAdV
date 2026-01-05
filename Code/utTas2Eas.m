function Eas = utTas2Eas(tas, alt)
    %tas, Eas(%equivalent vitesse) en m/s
    %alt en m
    %cf. http://www.hochwarth.com/misc/AviationCalculator.html
    Eas = sqrt(utRho(alt)/utRho(0))*tas;
end 