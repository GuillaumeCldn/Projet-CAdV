function tas = utEas2tas(Eas, alt)
    %tas, Eas (%equivalent vitesse) en m/s
    %alt en m
    %cf. http://www.hochwarth.com/misc/AviationCalculator.html
    tas = sqrt(utRho(0)/utRho(alt))*Eas;
end 