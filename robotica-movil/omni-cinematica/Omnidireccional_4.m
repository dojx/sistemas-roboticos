%% Actividad 5: Omnidireccional de 4 llantas
% Nombre: Diego Omar Jimenez Navarro
% Codigo: 213526346

%%
close all; clear; clc;

L = 0.25; % Distancia del centro del robot al centro de las ruedas (eje 'y' local)
l = 0.20; % Distancia del centro del robot al centro de las ruedas (eje 'x' local)

p = [-0.2 -0.2 pi]'; % Vector de posiciones generalizadas
pp = [0.0 0.0 0.0]'; % Vector de velocidades generalizadas

t = 0.01; % Tiempo de integracion
S = 5; % Tiempo total de simulacion

% Cinematica inversa
% Aplicamos cinematica inversa para encontrar las velocidades lineales
% necesarias en cada llanta para obtener las velocidades deseadas
% en el marco de referencia global
alpha = p(3) + (pi / 4); % Orientacion inicial
Ji = [sqrt(2)*sin(alpha) -sqrt(2)*cos(alpha) -(L+1); ...
        sqrt(2)*cos(alpha) sqrt(2)*sin(alpha) (L+1); ...
        sqrt(2)*cos(alpha) sqrt(2)*sin(alpha) -(L+1); ...
        sqrt(2)*sin(alpha) -sqrt(2)*cos(alpha) (L+1)];
q = [0.3 0.3 pi/8]'; % Velocidades deseadas en el marco de referencia global
v = Ji * q; % Velocidades lineales en cada llanta

% v = [-1 1 -1 1]'; % Velocidades lineales en cada llanta

p_plot = zeros(3, (S / t)); % Matriz que guardara las posiciones generalizadas en cada paso
time = t : t : S; % Vector de tiempo para las graficas de las posiciones generalizadas

for i = 1 : (S / t)
    % Cinematica directa
    alpha = p(3) + (pi / 4);
    
    % Matriz Jacobiana
    J = (1 / 4) * [sqrt(2)*sin(alpha) sqrt(2)*cos(alpha) sqrt(2)*cos(alpha) sqrt(2)*sin(alpha); ...
                    -sqrt(2)*cos(alpha) sqrt(2)*sin(alpha) sqrt(2)*sin(alpha) -sqrt(2)*cos(alpha); ...
                    -1/(L+1) 1/(L+1) -1/(L+1) 1/(L+1)];
                
    pp = J * v; % Actualizar velocidades
    p = p + pp * t; % Actualizar posiciones
    
    p_plot(:, i) = p; % Guardar posiciones actuales
    
    % Animacion
    clf
    Dibujar_Omnidireccional_4(p, L, l) 
    pause(t)
end

% Plotear los cambios de las posiciones generalizadas
figure
hold on

subplot(1, 3, 1)
plot(time, p_plot(1, :), 'r') % Cambios en x
grid on
title('x')

subplot(1, 3, 2)
plot(time, p_plot(2, :), 'g') % Cambios en y
grid on
title('y')

subplot(1, 3, 3)
plot(time, p_plot(3, :), 'b') % Cambios en theta
grid on
title('\theta')