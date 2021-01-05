clear all
close all
clc

% Parametros del omnidireccional
L = 0.1981;
l = 0.1990;

% Ganancias
kp = [12 12 6]'; % Proporcionales
ki = [0.0 0.0 0.0]'; % Integrales
kd = [0.0 0.0 0.0]'; % Derivativas

% Errores
e = [0.0 0.0 0.0]';
e_old = e;

% Errores para PID
ep = e;
ei = e;
ed = e;

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
    [~,OmniPlatform] = VREP.simxGetObjectHandle(ClientID,'OmniPlatform',VREP.simx_opmode_oneshot_wait);
    [~,motor_v1] = VREP.simxGetObjectHandle(ClientID,'OmniWheel_regularRotation#1',VREP.simx_opmode_oneshot_wait);
    [~,motor_v2] = VREP.simxGetObjectHandle(ClientID,'OmniWheel_regularRotation#0',VREP.simx_opmode_oneshot_wait);
    [~,motor_v3] = VREP.simxGetObjectHandle(ClientID,'OmniWheel_regularRotation#2',VREP.simx_opmode_oneshot_wait);
    [~,motor_v4] = VREP.simxGetObjectHandle(ClientID,'OmniWheel_regularRotation',VREP.simx_opmode_oneshot_wait);
    
    tic % Empezar reloj
	
    %Mientras la simulacion este activa hay que correr el bucle
    while VREP.simxGetConnectionId(ClientID)~=-1
        [~,position] = VREP.simxGetObjectPosition(ClientID,OmniPlatform,-1,VREP.simx_opmode_streaming);
        [~,orientation] = VREP.simxGetObjectOrientation(ClientID,OmniPlatform,-1,VREP.simx_opmode_streaming);
        [~,linearVelocity,angularVelocity] = VREP.simxGetObjectVelocity(ClientID,OmniPlatform,VREP.simx_opmode_streaming);
        
        p = [position(1) position(2) orientation(3)]'; % Tener cuidado de interpretar orientation(3) correctamente!
        pp = [linearVelocity(1) linearVelocity(2) angularVelocity(3)]';
        alpha = p(3) + pi/4;
        
        tt = toc; % Tiempo que ha pasado 
        
        % Puntos deseados
        z = 0.1 * tt;
        if (0 < z) && (z <= 2.5)
                xd = z; 
                yd = 0;
        elseif (2.5 < z) && (z <= 5)
                xd = 2.5; 
                yd = z - 2.5;
        elseif (5 < z) && (z <= 10)
                xd = 7.5 - z; 
                yd = 2.5;
        elseif (10 < z) && (z <= 15)
                xd = -2.5; 
                yd = 12.5 - z;
        elseif (15 < z) && (z <= 20)
                xd = z - 17.5; 
                yd = -2.5;
        elseif (20 < z) && (z <= 22.5)
                xd = 2.5; 
                yd = z - 22.5;
        elseif (22.5 < z) && (z <= 25)
                xd = 25 - z; 
                yd = 0;    
        else
                xd = 0; 
                yd = 0;
        end
        
        % Calcular errores de posicion
        pd = [xd yd 0]';
        e = pd - p;
        
        % Acotar theta entre -pi y pi
        if e(3) < -pi
            e(3) = -pi;
        end
        if e(3) > pi
            e(3) = pi;
        end
        
        % Errores para PID
        ep = e;
        ei = ei + e * t;
        ed = (e - e_old) / t;
        
        % Controlador PID
        % Calcular fuerzas
        u = kp .* e + ki .* ei + kd .* ed;
        
        e_old = e;
        
        % Velocidad para cada llanta
        v1 = sqrt(2)*sin(alpha)*u(1) - sqrt(2)*cos(alpha)*u(2) - (L+l)*u(3);
        v2 = sqrt(2)*cos(alpha)*u(1) + sqrt(2)*sin(alpha)*u(2) + (L+l)*u(3);
        v3 = sqrt(2)*cos(alpha)*u(1) + sqrt(2)*sin(alpha)*u(2) - (L+l)*u(3);
        v4 = sqrt(2)*sin(alpha)*u(1) - sqrt(2)*cos(alpha)*u(2) + (L+l)*u(3);
        
        %Cambiar la velocidad de los motores
        VREP.simxSetJointTargetVelocity(ClientID,motor_v1,v1,VREP.simx_opmode_streaming);
        VREP.simxSetJointTargetVelocity(ClientID,motor_v2,-v2,VREP.simx_opmode_streaming);
        VREP.simxSetJointTargetVelocity(ClientID,motor_v3,v3,VREP.simx_opmode_streaming);
        VREP.simxSetJointTargetVelocity(ClientID,motor_v4,-v4,VREP.simx_opmode_streaming);
        
        % Actualizar matrices para graficas
        p_plot = [p_plot p];
        pp_plot = [pp_plot pp];
        pd_plot = [pd_plot pd];
        q_plot = [q_plot u];
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
plot(t_plot, pd_plot(3, :), 'b', 'LineWidth', 2);
xlabel('t')
legend('x_d', 'y_d', '\theta_d')
title('Posiciones deseadas')

figure(5)
hold on
grid on
plot(t_plot, q_plot(1, :), 'r', 'LineWidth', 2);
plot(t_plot, q_plot(2, :), 'g', 'LineWidth', 2);
plot(t_plot, q_plot(3, :), 'b', 'LineWidth', 2);
xlabel('t')
legend('u_x', 'u_y', 'u_z')
title('Fuerzas')