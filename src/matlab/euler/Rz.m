
function R = Rz(psi)
%Rz realiza una rotación en un ángulo theta en radianes con respecto al eje z

R = [cos(psi) (-sin(psi)) 0
    sin(psi) cos(psi) 0
    0 0 1]
