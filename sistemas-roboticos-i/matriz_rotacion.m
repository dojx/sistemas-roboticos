%% Actividad 3
clc; clear all; close all;

w = [1 0 0]'; % Eje de rotacion
theta = pi/4; % Angulo que se rotara en el eje

%% Formula de Rodrigues
I = eye(3, 3); % Matriz identidad de 3x3
J = @(w) [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0]; % Matriz antisimetrica

% Matriz de rotacion
R1 = I + sin(theta)*J(w) + (1-cos(theta))*J(w)^2 % Formula de Rodrigues(1)
R2 = I + sin(theta)*J(w) + (1-cos(theta))*(w*w' - I) % Formula de Rodrigues(2)

%% Cuaternion
u0 = cos(theta/2); % Parte escalar del cuaternion
u = sin(theta/2)*w; % Parte vector del cuaternion
q = [u0; u]; % Concatenacion de partes

qUni = q(1)^2 + q(2:4)'*q(2:4) % Si es igual a uno es unitario

R3 = (q(1)^2 - q(2:4)'*q(2:4))*I + (2*q(1)*J(q(2:4))) + 2*(q(2:4)*q(2:4)') % Matriz de rotacion