%-------------------------
function y=utTemp(h)
    %temperature a l'altitude h
    %h en [m]
    %y en K
    beta_T = -6.5/1000; %K/m
    T0 = 288.15; %K
    y = T0 + beta_T*h;
end
