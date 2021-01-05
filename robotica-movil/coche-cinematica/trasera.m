%% Actividad 5 - Traccion trasera
% Nombre: Diego Omar Jimenez Navarro
% Codigo: 213526346
%%
close all; clear; clc;

vs = 0.4; % Velocidad lineal
wa = -0.25; % Velocidad angular
d = 0.3; % Distancia entre llantas traseras y delanteras

p = [-1 -1 0 pi/4]'; % Vector de posiciones generalizadas
pp = [0 0 0 0]'; % Vector de velocidades generalizadas

t = 0.01; % Step size
S = 5; % Tiempo total de simulacion

p_plot = zeros(4, (S / t)); % Matriz que guardara las posiciones generalizadas en cada paso
time = t : t : S; % Vector de tiempo para las graficas de las posiciones generalizadas

% Animacion de diferencial
for i = 1 : (S/t)
    % Dibujar
    clf
    Dibujar_Coche(p, d)
    pause(t)
    
    % Calcular velocidades generalizadas
    pp(1) = vs * cos(p(3)); % Velocidad x
    pp(2) = vs * sin(p(3)); % Velocidad y
    pp(3) = (vs / d) * tan(p(4)); % Velocidad theta
    pp(4) = wa; % Velocidad alpha
    
    p = p + pp * t; % Actualizar vector de posiciones
    
    p_plot(:, i) = p; % Guardar posiciones actuales
end

% Plotear los cambios de las posiciones generalizadas
figure
hold on

subplot(2, 2, 1)
plot(time, p_plot(1, :), 'r') % Cambios en x
grid on
title('x')

subplot(2, 2, 2)
plot(time, p_plot(2, :), 'g') % Cambios en y
grid on
title('y')

subplot(2, 2, 3)
plot(time, p_plot(3, :), 'b') % Cambios en theta
grid on
title('\theta')

subplot(2, 2, 4)
plot(time, p_plot(4, :), 'k') % Cambios en alpha
grid on
title('\alpha')