 %% Actividad 8
close all; 
clear; clc;

%% Posiciones finales deseadas (descomentar)
d = [1.5; 1.5]; % pf1
% d = [-1.5; 1.5]; % pf2
% d = [1.5; -1.5]; % pf3
% d = -[1.5; 1.5]; % pf4

%% Inicializacion
p = [0 0 0]'; % Posiciones iniciales

t = 0.01; % Tiempo de integracion
S = 20; % Tiempo total de simulacion

kp = [1; 2]; % Ganancias proporcionales
ki = [0; 0.1]; % Ganancias integrales
kd = [0.01; 0.01]; % Ganancias derivativas

e = [0; 0]; % Error actual
e_sum = [0; 0]; % Suma de errores
e_old = [0; 0]; % Error del paso anterior

% Matrices para guardar la simulacion
u_plot = zeros(2, S / t); % Salidas del controlador (v y w)
pp_plot = zeros(3, S / t); % Velocidades
p_plot = zeros(3, S / t); % Posiciones
time = t : t : S; % Vector de tiempo para las graficas

%% Simulacion
for i = 1 : (S / t)
%     clf
%     Dibujar_Movil(p)
%     pause(t)
    e_old = e; % Error anterior
    
    % Calcular error
    e(1) = sqrt((d(1) - p(1))^2 + (d(2) - p(2))^2); % Error en rho (distancia)
    % Error en theta (angulo)
    theta_d = atan2(d(2) - p(2), d(1) - p(1)); 
    e(2) = theta_d - p(3); 
    e(2) = atan2(sin(e(2)), cos(e(2))); % Acotar error entre -pi y pi
    
    % Suma total del error (para la parte integral)
    e_sum = e_sum + e * t;
    
    % Ley de control PID (calcula las fuerzas necesarias)
    u = (kp .* e) + (ki .* e_sum) + (kd .* (e - e_old))/t;
    
    % Matriz Jacobiana
    J = [cos(p(3)) 0; sin(p(3)) 0; 0 1];
    
    pp = J * u; % Actualizar velocidades
    p = p + pp * t; % Actualizar posiciones
    
    % Guardar la simulacion para despues graficar
    u_plot(:, i) = u;
    pp_plot(:, i) = pp;
    p_plot(:, i) = p;
end

%% Graficar simulacion
figure(1)
subplot(2, 2, 1)
plot(time, p_plot, 'LineWidth', 2);
grid on
legend('x', 'y', '\theta')
title('Posiciones p')

subplot(2, 2, 2)
plot(time, pp_plot, 'LineWidth', 2);
grid on
legend('xp', 'yp', '\thetap')
title('Velocidades p')

subplot(2, 2, 3)
plot(time, u_plot, 'LineWidth', 2);
grid on
legend('v', 'w')
title('Controlador')

subplot(2, 2, 4)
plot(p_plot(1, 1), p_plot(2, 1), 'ks')
hold on
plot(p_plot(1, end), p_plot(2, end), 'gs')
plot(p_plot(1, :), p_plot(2, :), 'LineWidth', 2)
grid on
legend('Punto inicial', 'Punto final')
title('Trayectoria')