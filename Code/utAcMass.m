function [mass] = utAcMass(kmass, OWE, MTOW)
    mass = OWE*(1-kmass) + kmass*(MTOW-OWE); %kg  
end  
