close all;
clear; clc;

figure(1)
simulacion([1.5 1.5])
figure(2)
simulacion([-1.5 1.5])
figure(3)
simulacion([1.5 -1.5])
figure(4)
simulacion([-1.5 -1.5])

function simulacion(d)
    D = 0.25; % Distancia del centro a la posicion de la rueda delantera
    alpha = 0; % Angulo de rotacion de la rueda delantera

    p = [0 0 0]'; % Posiciones iniciales
    pp = [0 0 0]'; % Velocidades iniciales

    t = 0.01; % Tiempo de integracion
    S = 20; % Tiempo de simulacion

    kp = 1; % Ganancia proporcional
    ki = 0; % Ganancia integral
    kd = 0.01; % Ganancia derivativa

    e = 0; % Error actual
    e_sum = 0; % Suma de errores
    e_old = 0; % Error del paso anterior

    % Matrices para guardar la simulacion
    u_plot = zeros(1, S / t); % Salidas del controlador (v y w)
    pp_plot = zeros(3, S / t); % Velocidades
    p_plot = zeros(3, S / t); % Posiciones
    time = t : t : S; % Vector de tiempo para las graficas

    % Simulacion
    for i = 1 : (S / t)
        e_old = e;  % Error anterior
        
        % Angulo deseado
        theta_d = atan2(d(2) - p(2), d(1) - p(1));
        alpha = atan2(sin(theta_d - p(3)), cos(theta_d - p(3)));
        
        % Restricciones de alpha
        if -pi/3 > alpha
            alpha = -pi/3;
        end        
        if pi/3 < alpha
            alpha = pi/3;
        end
        
        e = sqrt((d(1) - p(1))^2 + (d(2) - p(2))^2); % Error en rho (distancia)
        
        % Suma total del error (para la parte integral)
        e_sum = e_sum + e * t;

        % Ley de control PID (calcula las fuerza necesaria vs)
        vs = (kp * e) + (ki * e_sum) + (kd * (e - e_old))/t;
        
        % Calcular velocidades generalizadas
        pp(1) = vs * cos(alpha) * cos(p(3)); % Velocidad en x
        pp(2) = vs * cos(alpha) * sin(p(3)); % Velocidad en y
        pp(3) = (vs/D) * sin(alpha); % Velocidad de theta

        p = p + pp * t; % Actualizar posiciones

        % Guardar la simulacion para despues graficar
        u_plot(:, i) = vs;
        pp_plot(:, i) = pp;
        p_plot(:, i) = p;
    end
    
    % Graficar simulacion
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
    legend('vs')
    title('Controlador')

    subplot(2, 2, 4)
    plot(p_plot(1, 1), p_plot(2, 1), 'ks')
    hold on
    plot(p_plot(1, end), p_plot(2, end), 'gs')
    plot(p_plot(1, :), p_plot(2, :), 'LineWidth', 2)
    grid on
    legend('Punto inicial', 'Punto final')
    title('Trayectoria')
end