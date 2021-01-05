%% Actividad 3
% Nombre: Diego Omar Jimenez Navarro
% Codigo: 213526346

%%
close all; clear; clc;

% Funcion para calcular el modelo Jacobiano
f_J = @(tht) [cos(tht) 0; sin(tht) 0; 0 1];

p = [-0.5 -0.5 pi/2]'; % Vector de posiciones generalizadas
pp = [0 0 0]'; % Vector de velocidades generalizadas 

qp = [0.5; -pi/4]; % Velocidad lineal (m/s) y velocidad angular (rad/s)

S = 10; % Tiempo total que se simulara el robot

t = 0.01; % Incremento de tiempo (step size)

for i = 1 : (S/t)
    clf % Borrar figura
    Dibujar_Movil(p) % Dibujar robot en la posicion especificada
    pause(t) % Pausar la animacion por un momento
    
    % Calcular el modelo Jacobiano con el vector de variables de actuacion
    J = f_J(p(3)); 
    
    % Actualizar el vector de velocidades generalizadas
    pp = J * qp; 
    
    % Actualizar el vector de posiciones generalizadas
    p = p + pp * t; 
end

% Imprimir pose final del robot
p