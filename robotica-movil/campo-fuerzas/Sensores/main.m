clear all
close all
clc

R = 0.0975;
L = 0.381;

d_max = 0.9;
d_min = 0.1;

p_plot = [];

usensors = [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1];
utheta = [90,50,30,10,-10,-30,-50,-90,-90,-130,-150,-170,170,150,130,90]*(pi/180);

% Conectarse con el simulador
VREP = remApi('remoteApi'); % Crear el objeto vrep y cargar la libreria
ClientID = VREP.simxStart('127.0.0.1',19999,true,true,5000,5); % Iniciar conexión con el simulador

if ClientID~=-1
    %Crear un handle para los motores
    [~,Pioneer] = VREP.simxGetObjectHandle(ClientID,'Pioneer_p3dx',VREP.simx_opmode_oneshot_wait);
    [~,motorL] = VREP.simxGetObjectHandle(ClientID,'Pioneer_p3dx_leftMotor',VREP.simx_opmode_oneshot_wait);
    [~,motorR] = VREP.simxGetObjectHandle(ClientID,'Pioneer_p3dx_rightMotor',VREP.simx_opmode_oneshot_wait);
 
    for i=1:16
        [~,usensors(i)] = VREP.simxGetObjectHandle(ClientID,['Pioneer_p3dx_ultrasonicSensor' num2str(i)],VREP.simx_opmode_oneshot_wait);
    end
    
    figure

    %Mientras la simulacion este activa hay que correr el bucle
    while VREP.simxGetConnectionId(ClientID)~=-1
        [aux,position] = VREP.simxGetObjectPosition(ClientID,Pioneer,-1,VREP.simx_opmode_streaming);
        [~,orientation] = VREP.simxGetObjectOrientation(ClientID,Pioneer,-1,VREP.simx_opmode_streaming);
        
        p = [position(1) position(2) orientation(3)]';
        
        d_sensores = zeros(1,16);
        
        for i=1:16
            [~,res,d_si] = VREP.simxReadProximitySensor(ClientID,usensors(i),VREP.simx_opmode_streaming);
            d_sensores(i) = d_si(3);
            
            if res>0 && d_sensores(i) <=d_max
                if d_sensores(i)<d_min
                    d_sensores(i) = d_min;
                end
            else
                d_sensores(i) = 0;
            end 
        end
        
        d_sensores
        
        vr = 0;
        vl = 0;
        
        %Cambiar la velocidad de los motores
        VREP.simxSetJointTargetVelocity(ClientID,motorL,vl,VREP.simx_opmode_streaming);
        VREP.simxSetJointTargetVelocity(ClientID,motorR,vr,VREP.simx_opmode_streaming);
        
        p_plot = [p_plot p];
    end
end
 
VREP.simxFinish(ClientID); %Terminar la conexion

figure
hold on
grid on
plot(p_plot(1,:),p_plot(2,:),'b','LineWidth',2)
title('Trayectoria')
xlabel('x')
ylabel('y')
axis([-1 6 -1 6])

