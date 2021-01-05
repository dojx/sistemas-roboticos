clear all
close all
clc

R = 0.0975;
L = 0.381;

xg = 3;
yg = 3;

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

t = 0.005; % Tiempo de integracion (step size)

d_max = 0.9;
d_min = 0.1;

k_att = 1;
k_rep = 5;

h = 0.2;

p_plot = [];

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
    
    tic % Empezar reloj
 
    for i=1:16
        [~,usensors(i)] = VREP.simxGetObjectHandle(ClientID,['Pioneer_p3dx_ultrasonicSensor' num2str(i)],VREP.simx_opmode_oneshot_wait);
    end

    %Mientras la simulacion este activa hay que correr el bucle
    while VREP.simxGetConnectionId(ClientID)~=-1
        [aux,position] = VREP.simxGetObjectPosition(ClientID,Pioneer,-1,VREP.simx_opmode_streaming);
        [~,orientation] = VREP.simxGetObjectOrientation(ClientID,Pioneer,-1,VREP.simx_opmode_streaming);
        [~,linearVelocity,angularVelocity] = VREP.simxGetObjectVelocity(ClientID,Pioneer,VREP.simx_opmode_streaming);
        
        p = [position(1) position(2) orientation(3)]';
        pp = [linearVelocity(1) linearVelocity(2) angularVelocity(3)]';
        
        d_s = zeros(1,16);
        
        for i = 1:16
            [~,res,d_si] = VREP.simxReadProximitySensor(ClientID,usensors(i),VREP.simx_opmode_streaming);
            d_s(i) = d_si(3);
            
            if res>0 && d_s(i) <=d_max
                if d_s(i)<d_min
                    d_s(i) = d_min;
                end
            else
                d_s(i) = 0;
            end 
        end
        
        tt = toc; % Tiempo que ha pasado 
        
        % Campo de Fuerza
        f_att = k_att*[xg - p(1); yg - p(2)];
        
        f_rep = [0; 0];
        for i = 1 : 16
            f_rep = f_rep + k_rep*[(d_s(i)-d_max)*cos(utheta(i)); 
                                   (d_s(i)-d_max)*sin(utheta(i))];
        end
        
        f = f_att + f_rep;        
        n_f = f / sqrt(f(1)^2 + f(2)^2);
        
        % Puntos deseados
        xd = p(1) + h*n_f(1);
        yd = p(2) + h*n_f(2);
        
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
        VREP.simxSetJointTargetVelocity(ClientID,motorL,wl,VREP.simx_opmode_streaming);
        VREP.simxSetJointTargetVelocity(ClientID,motorR,wr,VREP.simx_opmode_streaming);
        
%         p_plot = [p_plot p];
    end
end
 
VREP.simxFinish(ClientID); %Terminar la conexion

% figure
% hold on
% grid on
% plot(p_plot(1,:),p_plot(2,:),'b','LineWidth',2)
% title('Trayectoria')
% xlabel('x')
% ylabel('y')
% axis([-1 6 -1 6])