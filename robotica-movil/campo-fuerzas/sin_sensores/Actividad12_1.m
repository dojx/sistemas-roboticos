% Actividad 12.1 Campo de Fuerza Artificial sin Sensores
% Sebastian Gonzalez
% 28/11/2020

clear all
close all
clc

R = 0.0975;
L = 0.381;

kp = [0.5; 0.5];

xg = 6.0;
yg = 6.0;

k_att = 1.0;
k_rep = 3.0;
phi = 1.8; %Radio
h = 0.2; %paso

p_plot = [];
pp_plot = [];
q_plot = [];
t_plot = [];

% Conectarse con el simulador
VREP = remApi('remoteApi'); % Crear el objeto vrep y cargar la libreria
ClientID = VREP.simxStart('127.0.0.1',19999,true,true,5000,5); % Iniciar conexión con el simulador

if ClientID~=-1
    %Crear un handle para los motores
    [~,Pioneer] = VREP.simxGetObjectHandle(ClientID,'Pioneer_p3dx',VREP.simx_opmode_oneshot_wait);
    [~,Obj1] = VREP.simxGetObjectHandle(ClientID,'Obj1',VREP.simx_opmode_oneshot_wait);
    [~,Obj2] = VREP.simxGetObjectHandle(ClientID,'Obj2',VREP.simx_opmode_oneshot_wait);
    [~,Obj3] = VREP.simxGetObjectHandle(ClientID,'Obj3',VREP.simx_opmode_oneshot_wait);
    [~,Obj4] = VREP.simxGetObjectHandle(ClientID,'Obj4',VREP.simx_opmode_oneshot_wait);
    [~,motorL] = VREP.simxGetObjectHandle(ClientID,'Pioneer_p3dx_leftMotor',VREP.simx_opmode_oneshot_wait);
    [~,motorR] = VREP.simxGetObjectHandle(ClientID,'Pioneer_p3dx_rightMotor',VREP.simx_opmode_oneshot_wait);
 
    tic
    figure
    
    %Mientras la simulacion este activa hay que correr el bucle
    while VREP.simxGetConnectionId(ClientID)~=-1
        [~,p_Obj1] = VREP.simxGetObjectPosition(ClientID,Obj1,-1,VREP.simx_opmode_streaming);
        [~,p_Obj2] = VREP.simxGetObjectPosition(ClientID,Obj2,-1,VREP.simx_opmode_streaming);
        [~,p_Obj3] = VREP.simxGetObjectPosition(ClientID,Obj3,-1,VREP.simx_opmode_streaming);
        [~,p_Obj4] = VREP.simxGetObjectPosition(ClientID,Obj4,-1,VREP.simx_opmode_streaming);
        [~,position] = VREP.simxGetObjectPosition(ClientID,Pioneer,-1,VREP.simx_opmode_streaming);
        [~,orientation] = VREP.simxGetObjectOrientation(ClientID,Pioneer,-1,VREP.simx_opmode_streaming);
        [~,linearVelocity,angularVelocity] = VREP.simxGetObjectVelocity(ClientID,Pioneer,VREP.simx_opmode_streaming);
        
        p = [position(1) position(2) orientation(3)]';
        pp = [linearVelocity(1) linearVelocity(2) angularVelocity(3)]';
        
        t = toc;
        
        [Fa_x,Fa_y] = f_atraccion(k_att,xg,yg,p(1),p(2));
        [Fr_x1,Fr_y1] = f_repulsion(k_rep,phi,p_Obj1(1),p_Obj1(2),p(1),p(2));
        [Fr_x2,Fr_y2] = f_repulsion(k_rep,phi,p_Obj2(1),p_Obj2(2),p(1),p(2));
        [Fr_x3,Fr_y3] = f_repulsion(k_rep,phi,p_Obj3(1),p_Obj3(2),p(1),p(2));
        [Fr_x4,Fr_y4] = f_repulsion(k_rep,phi,p_Obj4(1),p_Obj4(2),p(1),p(2));

        Fx = Fa_x + Fr_x1 + Fr_x2 + Fr_x3 + Fr_x4;
        Fy = Fa_y + Fr_y1 + Fr_y2 + Fr_y3 + Fr_y4;

        Fx_norm = Fx/(sqrt(Fx^2+Fy^2));
        Fy_norm = Fy/(sqrt(Fx^2+Fy^2));
        
        xd = p(1) + h*Fx_norm;
        yd = p(2) + h*Fy_norm;
        
        %Cálculo del error
        e(1) = sqrt((xd-p(1))^2 + (yd-p(2))^2); %Error de v
        theta_d = atan2((yd-p(2)),(xd-p(1)));
        e(2) = theta_d - p(3); %Error de w
        e(2) = atan2(sin(e(2)),cos(e(2))); %Acota a valores de -pi a pi
        
        %Acciones de control del PID
        v = kp(1)*e(1);
        w = kp(2)*e(2);

        %Velocidad angular de cada rueda
        vr = (2*v+w*L)/(2*R);
        vl = (2*v-w*L)/(2*R);
        
        if sqrt((xg-p(1))^2 + (yg-p(2))^2) < 0.05
            vr = 0;
            vl = 0;
        end
 
        %Cambiar la velocidad de los motores
        VREP.simxSetJointTargetVelocity(ClientID,motorL,vl,VREP.simx_opmode_streaming);
        VREP.simxSetJointTargetVelocity(ClientID,motorR,vr,VREP.simx_opmode_streaming);
    
        p_plot = [p_plot p];
        pp_plot = [pp_plot pp];
        q_plot = [q_plot [v; w]];
        t_plot = [t_plot t];
        
        cla
        hold on; grid on;
        plot(p_plot(1,:),p_plot(2,:),'b','LineWidth',2)
        plot(xg,yg,'og','LineWidth',2,'MarkerSize',10)
        plot(p(1),p(2),'^k','LineWidth',2,'MarkerSize',10)
        plot(p_Obj1(1),p_Obj1(2),'s','Color',[0.63 0.5 0.38],'LineWidth',20,'MarkerSize',5)
        plot(p_Obj2(1),p_Obj2(2),'s','Color',[0.63 0.5 0.38],'LineWidth',20,'MarkerSize',5)
        plot(p_Obj3(1),p_Obj3(2),'s','Color',[0.63 0.5 0.38],'LineWidth',20,'MarkerSize',5)
        plot(p_Obj4(1),p_Obj4(2),'s','Color',[0.63 0.5 0.38],'LineWidth',20,'MarkerSize',5)
        title('Trayectoria');
        xlabel('x');
        ylabel('y');
        axis([-1 7 -1 7])
        drawnow

    end
end
 
VREP.simxFinish(ClientID); %Terminar la conexion

% figure
% hold on;
% grid on;
% plot(t_plot,p_plot(1,:),'r','LineWidth',2)
% plot(t_plot,p_plot(2,:),'g','LineWidth',2)
% plot(t_plot,p_plot(3,:),'b','LineWidth',2)
% title('p');
% xlabel('t');
% ylabel('m, rad');
% legend('x','y','\theta','Location','best')
% 
% figure
% hold on;
% grid on;
% plot(t_plot,pp_plot(1,:),'m','LineWidth',2)
% plot(t_plot,pp_plot(2,:),'g','LineWidth',2)
% plot(t_plot,pp_plot(3,:),'k','LineWidth',2)
% title('p_p');
% xlabel('t');
% ylabel('m, rad');
% legend('x_p','y_p','\theta_p','Location','best')
% 
% figure
% hold on;
% grid on;
% plot(t_plot,q_plot(1,:),'m','LineWidth',2)
% plot(t_plot,q_plot(2,:),'g','LineWidth',2)
% title('q');
% xlabel('t');
% ylabel('m/s, rad/s');
% legend('v','w','Location','best')