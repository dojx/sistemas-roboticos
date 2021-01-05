%% Actividad 5: Omnidireccional de 3 llantas
%% Actividad 5: Omnidireccional de 3 llantas
% Nombre: Diego Omar Jimenez Navarro
% Codigo: 213526346

%%
close all; clear; clc;

L = 0.25; % Distancia del centro del robot al centro de cada rueda

p = [0.5 -1 pi/3]'; % Vector de posiciones generalizadas
pp = [0.0 0.0 0.0]'; % Vector de velocidades generalizadas

t = 0.01; % Tiempo de integracion
S = 5; % Tiempo total de simulacion

% Cinematica inversa
% Aplicamos cinematica inversa para encontrar las velocidades lineales
% necesarias en cada llanta para obtener las velocidades deseadas
% en el marco de referencia global
Ji = [-sin(p(3)) cos(p(3)) L; ...
        -sin(p(3)+(2*pi/3)) cos(p(3)+(2*pi/3)) L; ...
        -sin(p(3)+(4*pi/3)) cos(p(3)+(4*pi/3)) L]; 
q = [0.5 0.5 10]'; % Velocidades deseadas en el marco de referencia global
v = Ji * q; % Velocidades lineales en cada llanta

% v = [3 3 3]'; % Velocidades lineales en cada llanta

p_plot = zeros(3, (S / t)); % Matriz que guardara las posiciones generalizadas en cada paso
time = t : t : S; % Vector de tiempo para las graficas de las posiciones generalizadas

for i = 1 : (S / t)
    % Cinematica directa   
    % Matriz Jacobiana
    J = [(-2/3)*sin(p(3)) (-2/3)*sin(p(3)+(2*pi/3)) (-2/3)*sin(p(3)+(4*pi/3));
        (2/3)*cos(p(3)) (2/3)*cos(p(3)+(2*pi/3)) (2/3)*cos(p(3)+(4*pi/3));
        (1/3)*L (1/3)*L (1/3)*L];
    
    pp = J * v; % Actualizar velocidades
    p = p + pp * t; % Actualizar posiciones
    
    p_plot(:, i) = p; % Guardar posiciones actuales
    
    % Animacion
    clf
    Dibujar_Omnidireccional_3(p, L)
    pause(t);
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