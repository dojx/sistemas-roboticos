clear all
close all
clc

R = 0.0975;
L = 0.381;

% Deseado
xg = 3;
yg = 3;

% Ganancias
kp = [0.5 0.5]; % Proporcionales

% Errores
e = [0.0 0.0];
% Errores PID
ep = e;

t = 0.005; % Tiempo de integracion (step size)

% Distancia maxima sensor
d_max = 0.9;
d_min = 0.1;

% Ganancias de fuerzas
k_att = 1;
k_rep = 3;

% Fuerzas iniciales
f_att = [0; 0];
f_rep = [0; 0];
h = 0.2;

% Matrices para las graficas
p_plot = [];
pp_plot = [];
pd_plot = [];
q_plot = [];
t_plot = [];

usensors = [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1];
utheta = [90,50,30,10,-10,-30,-50,-90,-90,-130,-150,-170,170,150,130,90]*(pi/180);

% Conectarse con el simulador
VREP = remApi('remoteApi'); % Crear el objeto vrep y cargar la libreria
ClientID = VREP.simxStart('127.0.0.1',19999,true,true,5000,5); % Iniciar conexión con el simulador

if ClientID~=-1
    % Crear un handle para los motores
    [~,Pioneer] = VREP.simxGetObjectHandle(ClientID,'Pioneer_p3dx',VREP.simx_opmode_oneshot_wait);
    [~,motorL] = VREP.simxGetObjectHandle(ClientID,'Pioneer_p3dx_leftMotor',VREP.simx_opmode_oneshot_wait);
    [~,motorR] = VREP.simxGetObjectHandle(ClientID,'Pioneer_p3dx_rightMotor',VREP.simx_opmode_oneshot_wait);
    
    % Obstaculos
    [~,cuboid] = VREP.simxGetObjectHandle(ClientID,'ConcretBlock',VREP.simx_opmode_oneshot_wait);
    [~,cuboid0] = VREP.simxGetObjectHandle(ClientID,'ConcretBlock#0',VREP.simx_opmode_oneshot_wait);
    [~,cuboid1] = VREP.simxGetObjectHandle(ClientID,'ConcretBlock#1',VREP.simx_opmode_oneshot_wait);
    
    tic % Empezar reloj
 
    for i=1:16
        [~,usensors(i)] = VREP.simxGetObjectHandle(ClientID,['Pioneer_p3dx_ultrasonicSensor' num2str(i)],VREP.simx_opmode_oneshot_wait);
    end

    %Mientras la simulacion este activa hay que correr el bucle
    while VREP.simxGetConnectionId(ClientID)~=-1
        [aux,position] = VREP.simxGetObjectPosition(ClientID,Pioneer,-1,VREP.simx_opmode_streaming);
        [~,orientation] = VREP.simxGetObjectOrientation(ClientID,Pioneer,-1,VREP.simx_opmode_streaming);
        [~,linearVelocity,angularVelocity] = VREP.simxGetObjectVelocity(ClientID,Pioneer,VREP.simx_opmode_streaming);
        
        % Posiciones de obstaculos
        [~,o_position] = VREP.simxGetObjectPosition(ClientID,cuboid,-1,VREP.simx_opmode_streaming);
        [~,o_position0] = VREP.simxGetObjectPosition(ClientID,cuboid0,-1,VREP.simx_opmode_streaming);
        [~,o_position1] = VREP.simxGetObjectPosition(ClientID,cuboid1,-1,VREP.simx_opmode_streaming);
        op = [o_position(1) o_position(2)]';
        op0 = [o_position0(1) o_position0(2)]';
        op1 = [o_position1(1) o_position1(2)]';
        
        % Posicion actual
        p = [position(1) position(2) orientation(3)]';
        pp = [linearVelocity(1) linearVelocity(2) angularVelocity(3)]';
        
        d_s = zeros(1,16);
        f_rep = [0; 0];
        for i = 1:16
            [~,res,d_si] = VREP.simxReadProximitySensor(ClientID,usensors(i),VREP.simx_opmode_streaming);
            d_s(i) = d_si(3);
            
            % Checar si sensor detecto algo
            if res>0 && d_s(i) <=d_max
                if d_s(i)<d_min
                    d_s(i) = d_min;
                end
                % Suma de fuerzas de repulsion
                f_rep = f_rep + k_rep*[(d_s(i)-d_max)*cos(utheta(i)); 
                                   (d_s(i)-d_max)*sin(utheta(i))];
            else
                d_s(i) = 0;
            end 
        end
        
        tt = toc; % Tiempo que ha pasado 
        
        % Campo de Fuerza
        f_att = k_att*[xg - p(1); yg - p(2)];
        
        f = f_att + f_rep;        
        n_f = f / sqrt(f(1)^2 + f(2)^2);
        
        % Puntos deseados
        xd = p(1) + h*n_f(1);
        yd = p(2) + h*n_f(2);
        
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
        VREP.simxSetJointTargetVelocity(ClientID,motorL,wl,VREP.simx_opmode_streaming);
        VREP.simxSetJointTargetVelocity(ClientID,motorR,wr,VREP.simx_opmode_streaming);
        
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
plot(0, 0, 'ko', 'MarkerSize', 10);
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