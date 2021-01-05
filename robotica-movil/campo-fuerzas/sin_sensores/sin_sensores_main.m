clear all
close all
clc

R = 0.0975; % Radio de llantas
L = 0.381; % Distancia entre llantas

% Deseados finales
xg = 3;
yg = 3;

% Ganancias
kp = [0.5 0.5]; % Proporcionales

k_att = 1; % Ganancia de fuerza de atraccion
k_rep = 3; % Ganancia de fuerza de repulsion
phi = 1.5; % Distancia a que se aplican las fuerzas
h = 0.2; % Escalamiento de fuerza

% Errores
e = [0.0 0.0];

t = 0.005; % Tiempo de integracion (step size)

% Matrices para las graficas
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
    % Obstaculos
    [~,cuboid] = VREP.simxGetObjectHandle(ClientID,'Cuboid',VREP.simx_opmode_oneshot_wait);
    [~,cuboid0] = VREP.simxGetObjectHandle(ClientID,'Cuboid0',VREP.simx_opmode_oneshot_wait);
    [~,cuboid1] = VREP.simxGetObjectHandle(ClientID,'Cuboid1',VREP.simx_opmode_oneshot_wait);
    
    tic % Empezar reloj
 
    %Mientras la simulacion este activa hay que correr el bucle
    while VREP.simxGetConnectionId(ClientID) ~= -1
        % Obtener posiciones, orientaciones y velocidades
        [~,position] = VREP.simxGetObjectPosition(ClientID,Pioneer,-1,VREP.simx_opmode_streaming);

        [~,orientation] = VREP.simxGetObjectOrientation(ClientID,Pioneer,-1,VREP.simx_opmode_streaming);
        [~,linearVelocity,angularVelocity] = VREP.simxGetObjectVelocity(ClientID,Pioneer,VREP.simx_opmode_streaming);
        
        % Posiciones de obstaculos
        [~,o_position] = VREP.simxGetObjectPosition(ClientID,cuboid,-1,VREP.simx_opmode_streaming);
        [~,o_position0] = VREP.simxGetObjectPosition(ClientID,cuboid0,-1,VREP.simx_opmode_streaming);
        [~,o_position1] = VREP.simxGetObjectPosition(ClientID,cuboid1,-1,VREP.simx_opmode_streaming);
        op = [o_position(1) o_position(2)]';
        op0 = [o_position0(1) o_position0(2)]';
        op1 = [o_position1(1) o_position1(2)]';
        
        p = [position(1) position(2) orientation(3)]';
        pp = [linearVelocity(1) linearVelocity(2) angularVelocity(3)]';
        
        tt = toc; % Tiempo que ha pasado 
        
        % Campo de potencias
        [fa_x, fa_y] = f_atraccion(k_att, xg, yg, p(1), p(2));
        [fr_x1, fr_y1] = f_repulsion(k_rep, phi, op(1), op(2), p(1), p(2));
        [fr_x2, fr_y2] = f_repulsion(k_rep, phi, op0(1), op0(2), p(1), p(2));
        [fr_x3, fr_y3] = f_repulsion(k_rep, phi, op1(1), op1(2), p(1), p(2));
             
        fx = fa_x + fr_x1 + fr_x2 + fr_x3;
        fy = fa_y + fr_y1 + fr_y2 + fr_y3;

        n_fx = fx / sqrt(fx^2 + fy^2);
        n_fy = fy / sqrt(fx^2 + fy^2);
        
        % Puntos deseados
        xd = p(1) + h*n_fx;
        yd = p(2) + h*n_fy;
        
        % Calcular errores
        e(1) = sqrt((xd - p(1))^2 + (yd - p(2))^2);
        e(2) = atan2(yd - p(2), xd - p(1)) - p(3);
        e(2) = atan2(sin(e(2)), cos(e(2)));
        
        % Controlador PID
        u = kp .* e;
        
        % Velocidad en cada llanta
        wr = (2*u(1) + u(2)*L)/(2*R);
        wl = (2*u(1) - u(2)*L)/(2*R);
        
        if sqrt((xg-p(1))^2 + (yg-p(2))^2) < 0.05
            wr = 0;
            wl = 0;
        end
        
        %Cambiar la velocidad de los motores
        VREP.simxSetJointTargetVelocity(ClientID, motorL, wl, VREP.simx_opmode_streaming);
        VREP.simxSetJointTargetVelocity(ClientID, motorR, wr, VREP.simx_opmode_streaming);
        
        % Actualizar matrices para graficas
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
plot(p_plot(1, :), p_plot(2, :), 'b', 'LineWidth', 2);
plot(op(1), op(2), 'ro', 'MarkerSize', 10);
plot(op0(1), op0(2), 'ro', 'MarkerSize', 10);
plot(op1(1), op1(2), 'ro', 'MarkerSize', 10);
plot(xg, yg, 'go', 'MarkerSize', 10);
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