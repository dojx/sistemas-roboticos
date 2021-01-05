%% Actividad 7: Manipulador en omnidireccional
% close all; 
figure
clear; clc;

%% Funciones
% Matriz Jacobiana
J = @(q) [ 1, 0, - sin(q(3))/4 - (cos(q(3))*sin(q(4))*(5*cos(q(5) + q(6)) + 6*cos(q(5))))/20 - (cos(q(4))*sin(q(3))*(5*cos(q(5) + q(6)) + 6*cos(q(5))))/20, -(sin(q(3) + q(4))*(5*cos(q(5) + q(6)) + 6*cos(q(5))))/20, -(cos(q(3) + q(4))*(5*sin(q(5) + q(6)) + 6*sin(q(5))))/20, -(cos(q(3) + q(4))*sin(q(5) + q(6)))/4;
         0, 1,   cos(q(3))/4 + (cos(q(3))*cos(q(4))*(5*cos(q(5) + q(6)) + 6*cos(q(5))))/20 - (sin(q(3))*sin(q(4))*(5*cos(q(5) + q(6)) + 6*cos(q(5))))/20,  (cos(q(3) + q(4))*(5*cos(q(5) + q(6)) + 6*cos(q(5))))/20, -(sin(q(3) + q(4))*(5*sin(q(5) + q(6)) + 6*sin(q(5))))/20, -(sin(q(3) + q(4))*sin(q(5) + q(6)))/4;
         0, 0,                                                                                                                   0,                                               0,                 cos(q(5) + q(6))/4 + (3*cos(q(5)))/10,                 cos(q(5) + q(6))/4];

% Vector de traslacion (ultima columna de wTe)   
vt = @(q) [q(1) + cos(q(3))/4 + cos(q(3))*((3*cos(q(4))*cos(q(5)))/10 + (cos(q(4))*cos(q(5))*cos(q(6)))/4 - (cos(q(4))*sin(q(5))*sin(q(6)))/4) - sin(q(3))*((3*cos(q(5))*sin(q(4)))/10 - (sin(q(4))*sin(q(5))*sin(q(6)))/4 + (cos(q(5))*cos(q(6))*sin(q(4)))/4);
           q(2) + sin(q(3))/4 + sin(q(3))*((3*cos(q(4))*cos(q(5)))/10 + (cos(q(4))*cos(q(5))*cos(q(6)))/4 - (cos(q(4))*sin(q(5))*sin(q(6)))/4) + cos(q(3))*((3*cos(q(5))*sin(q(4)))/10 - (sin(q(4))*sin(q(5))*sin(q(6)))/4 + (cos(q(5))*cos(q(6))*sin(q(4)))/4);
           (3*sin(q(5)))/10 + (cos(q(5))*sin(q(6)))/4 + (cos(q(6))*sin(q(5)))/4 + 3/5];
       
%% Simulacion
p_base = [0 0 0]'; % Coordenadas generalizadas robot movil
q_arm = [0 pi/4 -pi/4]'; % Coordenadas generalizadas manipulador
q = [p_base; q_arm]; % Coordenadas generalizadas
K = [1 0 0; 0 1 0; 0 0 1]; % Matriz de ganancias

xd = [1 1 0.5]'; % Deseados

t = 0.01; % Tiempo de integracion
S = 10; % Tiempo total de simulacion

x_plot = zeros(3, (S / t)); % Matriz que guardara las posiciones globales en cada paso
xp_plot = zeros(3, (S / t)); % Matriz que guardara las velocidades(errores) globales en cada paso
q_plot = zeros(6, (S / t)); % Matriz que guardara las posiciones del robot movil y el manipulador en cada paso
qp_plot = zeros(6, (S / t)); % Matriz que guardara las velocidades(errores) del robot movil y el manipulador en cada paso
time = t : t : S; % Vector de tiempo para las graficas

% Simulacion
for i = 1 : (S / t)
    x = vt(q); % Calcular posiciones actuales
    
    err = xd - x; % Error
    J_inv = pinv(J(q)); % Inversa del Jacobiano (pseudo-inversa)
    
    qp = J_inv * (K * err); % Actualizar velocidades
    q = q + qp * t; % Actualizar posiciones
    
    x_plot(:, i) = x; % Guardar posiciones actuales    
    xp_plot(:, i) = err; % Guardar errores actuales
    
    q_plot(:, i) = q; % Guardar posiciones actuales (movil y manipulador)   
    qp_plot(:, i) = qp; % Guardar errores actuales (movil y manipulador)
end

%% Graficas
figure(1)
subplot(2, 3, 1)
hold on
grid on
plot(time, x_plot(1, :), 'r')
plot(time, ones(size(time)) * xd(1), '--m', 'LineWidth', 2)
title('x')
legend('x', 'Deseado')

subplot(2, 3, 2)
hold on
grid on
plot(time, x_plot(2, :), 'b')
plot(time, ones(size(time)) * xd(2), '--c', 'LineWidth', 2)
title('y')
legend('y', 'Deseado')

subplot(2, 3, 3)
hold on
grid on
plot(time, x_plot(3, :), 'g')
plot(time, ones(size(time)) * xd(3), '--k', 'LineWidth', 2)
title('z')
legend('z', 'Deseado')

subplot(2, 3, 4)
hold on
grid on
plot(time, xp_plot(1, :), 'r')
plot(time, zeros(size(time)), '--m', 'LineWidth', 2)
title('xp')
legend('xp', 'Deseado')

subplot(2, 3, 5)
hold on
grid on
plot(time, xp_plot(2, :), 'b')
plot(time, zeros(size(time)), '--c', 'LineWidth', 2)
title('yp')
legend('yp', 'Deseado')

subplot(2, 3, 6)
hold on
grid on
plot(time, xp_plot(3, :), 'g')
plot(time, zeros(size(time)), '--k', 'LineWidth', 2)
title('zp')
legend('zp', 'Deseado')

figure(2)
subplot(1, 2, 1)
plot(time, q_plot, 'LineWidth', 2)
grid on
legend('p_1', 'p_2', 'p_3', 'q_1', 'q_2', 'q_3')

subplot(1, 2, 2)
plot(time, qp_plot, 'LineWidth', 2)
grid on
legend('p_1_p', 'p_2_p', 'p_3_p', 'q_1_p', 'q_2_p', 'q_3_p');