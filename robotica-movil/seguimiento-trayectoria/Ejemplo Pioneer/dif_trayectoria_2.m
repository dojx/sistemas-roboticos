clear all
close all
clc

R = 0.0975; % Radio de llantas
L = 0.381; % Distancia entre llantas

% Ganancias
kp = [0.5 2]; % Proporcionales
ki = [0.0 0.0]; % Integrales
kd = [0.0 0.0]; % Derivativas

% Errores
e = [0.0 0.0];
e_old = e;

% Errores PID
ep = e;
ei = e;
ed = e;

% Parametros para trayectoria
dx = 0.5;
dy = dx;
r = 1;

t = 0.005; % Tiempo de integracion (step size)

% Matrices para graficas
p_plot = [];
pp_plot = [];
pd_plot = [];
q_plot = [];
t_plot = [];

% Conectarse con el simulador
VREP = remApi('remoteApi'); % Crear el objeto vrep y cargar la libreria
ClientID = VREP.simxStart('127.0.0.1',19999,true,true,5000,5); % Iniciar conexión con el simulador

if ClientID~=-1
    %Crear un handle para los motores
    [~,Pioneer] = VREP.simxGetObjectHandle(ClientID,'Pioneer_p3dx',VREP.simx_opmode_oneshot_wait);
    [~,motorL] = VREP.simxGetObjectHandle(ClientID,'Pioneer_p3dx_leftMotor',VREP.simx_opmode_oneshot_wait);
    [~,motorR] = VREP.simxGetObjectHandle(ClientID,'Pioneer_p3dx_rightMotor',VREP.simx_opmode_oneshot_wait);
    
    tic % Empezar reloj
 
    %Mientras la simulacion este activa hay que correr el bucle
    while VREP.simxGetConnectionId(ClientID) ~= -1
        % Obtener posiciones, orientaciones y velocidades
        [~,position] = VREP.simxGetObjectPosition(ClientID,Pioneer,-1,VREP.simx_opmode_streaming);
        [~,orientation] = VREP.simxGetObjectOrientation(ClientID,Pioneer,-1,VREP.simx_opmode_streaming);
        [~,linearVelocity,angularVelocity] = VREP.simxGetObjectVelocity(ClientID,Pioneer,VREP.simx_opmode_streaming);
        
        p = [position(1) position(2) orientation(3)]';
        pp = [linearVelocity(1) linearVelocity(2) angularVelocity(3)]';
        
        tt = toc; % Tiempo que ha pasado 
        
        % Puntos deseados
        xd = dx + r * cos(tt * 0.1);
        yd = dy + r * sin(tt * 0.1);
        
        % Calcular errores
        e(1) = sqrt((xd - p(1))^2 + (yd - p(2))^2);
        e(2) = atan2(yd - p(2), xd - p(1)) - p(3);
        e(2) = atan2(sin(e(2)), cos(e(2)));
        
        % Errores para PID
        ep = e;
        ei = ei + e * t;
        ed = (e - e_old) / t;
        
        % Controlador PID
        u = (kp .* e) + (ki .* ei) + (kd .* ed);
        
        % Actualizar error
        e_old = e;
        
        % Velocidad en cada llanta
        wr = (2*u(1) + u(2)*L)/(2*R);
        wl = (2*u(1) - u(2)*L)/(2*R);
        
        %Cambiar la velocidad de los motores
        VREP.simxSetJointTargetVelocity(ClientID, motorL, wl, VREP.simx_opmode_streaming);
        VREP.simxSetJointTargetVelocity(ClientID, motorR, wr, VREP.simx_opmode_streaming);
        
        % Actualizar graficas
        p_plot = [p_plot p];
        pp_plot = [pp_plot pp];
        pd_plot = [pd_plot [xd; yd]];
        q_plot = [q_plot [u(1); u(2)]];
        t_plot = [t_plot tt];
    end
end
 
VREP.simxFinish(ClientID); %Terminar la conexion

%% Graficas
figure(1)
hold on
grid on
plot(pd_plot(1, :), pd_plot(2, :), 'r', 'LineWidth', 2);
plot(p_plot(1, :), p_plot(2, :), 'b', 'LineWidth', 2);
xlabel('x')
ylabel('y')
title('Trayectoria')

figure(2)
hold on
grid on
plot(t_plot, p_plot(1, :), 'r', 'LineWidth', 2);
plot(t_plot, p_plot(2, :), 'g', 'LineWidth', 2);
plot(t_plot, p_plot(3, :), 'b', 'LineWidth', 2);
xlabel('t')
legend('x', 'y', '\theta')
title('Posiciones')

figure(3)
hold on
grid on
plot(t_plot, pp_plot(1, :), 'r', 'LineWidth', 2);
plot(t_plot, pp_plot(2, :), 'g', 'LineWidth', 2);
plot(t_plot, pp_plot(3, :), 'b', 'LineWidth', 2);
xlabel('t')
legend('x_p', 'y_p', '\theta_p')
title('Velocidades')

figure(4)
hold on
grid on
plot(t_plot, pd_plot(1, :), 'r', 'LineWidth', 2);
plot(t_plot, pd_plot(2, :), 'g', 'LineWidth', 2);
xlabel('t')
legend('x_d', 'y_d')
title('Posiciones deseadas')

figure(5)
hold on
grid on
plot(t_plot, q_plot(1, :), 'r', 'LineWidth', 2);
plot(t_plot, q_plot(2, :), 'g', 'LineWidth', 2);
xlabel('t')
legend('v', 'w')
title('Fuerzas')