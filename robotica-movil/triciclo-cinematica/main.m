%% Actividad 4 - Triciclo
% Nombre: Diego Omar Jimenez Navarro
% Codigo: 213526346
%%
close all; clear; clc;

vs = 0.3; % Velocidad lineal de la llanta delantera
alpha = pi / 5; % Angulo de rotacion de la rueda delantera
d = 0.2; % Distancia del centro a la posicion de la rueda delantera

p = [-0.4 0.5 -pi/3]'; % Vector de posiciones generalizadas
pp = [0.0 0.0 0.0]'; % Vector de velocidades generalizadas

t = 0.01; % Incremento de tiempo (step size)
S = 5; % Tiempo total que se simulara el robot

p_plot = zeros(3, (S / t)); % Matriz que guardara las posiciones generalizadas en cada paso
time = t : t : S; % Vector usado para las graficas de las posiciones generalizadas

% Animacion de Triciclo
for i = 1 : (S/t)
    % Dibujar
    clf
    Dibujar_Triciclo(p, alpha, d)
    pause(t)
    
    % Calcular velocidades generalizadas
    pp(1) = vs * cos(alpha) * cos(p(3)); % Velocidad x
    pp(2) = vs * cos(alpha) * sin(p(3)); % Velocidad y
    pp(3) = (vs / d) * sin(alpha); % Velocidad theta
    
    % Calcular posiciones generalizadas
    p = p + pp * t;
    
    % Guardar valores en la matriz para plotear
    p_plot(:, i) = p;
end

% Plotear los cambios de las posiciones generalizadas
figure
hold on
subplot(3, 1, 1)
plot(time, p_plot(1, :), 'r') % Posicion x
title('x')
subplot(3, 1, 2)
plot(time, p_plot(2, :), 'g') % Posicion y
title('y')
subplot(3, 1, 3)
plot(time, p_plot(3, :), 'b') % Angulo theta
title('\theta')