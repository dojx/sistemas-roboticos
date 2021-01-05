%% Actividad 4 - Diferencial
% Nombre: Diego Omar Jimenez Navarro
% Codigo: 213526346
%%
close all; clear; clc;

vl = 0.3; % Velocidad lineal llanta izquierda
vr = 0.3; % Velocidad lineal llanta derecha
v = (vl + vr) / 2; % Velocidad lineal del diferencial (promedio)

L = 0.3; % Distancia entre ambas llantas

p = [0.5 -0.5 pi/6]'; % Vector de posiciones generalizadas
pp = [0.0 0.0 0.0]'; % Vector de velocidades generalizadas

t = 0.01; % Incremento de tiempo (step size)
S = 5; % Tiempo de simulacion 

p_plot = zeros(3, (S / t)); % Matriz que guardara las posiciones generalizadas en cada paso
time = t : t : S; % Vector usado para las graficas de las posiciones generalizadas

% Animacion de diferencial
for i = 1 : (S/t)
    % Dibujar
    clf
    Dibujar_Diferencial(p, L)
    pause(t)
    
    % Calcular velocidades generalizadas
    pp(1) = v * cos(p(3)); % Velocidad en x
    pp(2) = v * sin(p(3)); % Velocidad en y
    pp(3) = (vr - vl) / L; % Velocidad de theta
    
    p = p + pp * t; % Actualizar vector de posiciones
    
    p_plot(:, i) = p; % Guardar posiciones actuales
end

% Plotear los cambios de las posiciones generalizadas
figure
hold on
subplot(3, 1, 1)
plot(time, p_plot(1, :), 'r') % Cambios en x
title('x')
subplot(3, 1, 2)
plot(time, p_plot(2, :), 'g') % Cambios en y
title('y')
subplot(3, 1, 3)
plot(time, p_plot(3, :), 'b') % Cambios en theta
title('\theta')