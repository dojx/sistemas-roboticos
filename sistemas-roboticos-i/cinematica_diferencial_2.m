clear all; close all; clc

% Eslabones medidas
a1 = 0.15;
a2 = 0.40;
d1 = 0.80;
d4 = 0.20;

% Articulaciones
L1 = Revolute('a',a1,'alpha',0,'d',d1,'offset',0);
L2 = Revolute('a',a2,'alpha',pi,'d',0,'offset',0);
L3 = Prismatic('a',0,'alpha',0,'theta',0,'offset',0);
L4 = Revolute('a',0,'alpha',0,'d',d4,'offset',0);
bot = SerialLink([L1 L2 L3 L4],'name','SCARA 4DOF'); % Unir articulaciones

q = [pi/4 pi/4 1 pi/4]'; % Vector de coordenadas generalizadas

caso = 1; % (1, 2, 3)
posicion = 1; % Numero de posicion deseada

switch posicion
    case 1
        td = [0.3889 0.3889 0.4]';%Posicion deseada
        phi = -pi/2;%Factor para la rotacion deseada
    case 2
        td = [0.5 0.2 0.1]';
        phi = pi/2;          
    otherwise
        return;
end

Rd = [cos(phi) sin(phi) 0; sin(phi) -cos(phi) 0; 0 0 -1];%Rotacion deseada
k = eye(6); % Matriz de control

t = 0.01 ;% Tiempo de muestreo
S = 10; % Tiempo de simulacion

for i=1:S/t

    T04 = bot.fkine(q); % Cinematica directa
                                                                
    J = [ -a2*sin(q(1)+q(2))-a1*sin(q(1)), -a2*sin(q(1)+q(2)), 0, 0;
          a2*cos(q(1)+q(2))+a1*cos(q(1)), a2*cos(q(1)+q(2)), 0, 0;
          0, 0, -1, 0;
          0, 0, 0, 0;
          0, 0, 0, 0;
          1, 1, 0, -1]; % Matriz jacobiana

    v = td-T04(1:3,4); % Velocidades articulaciones
    w = (cross(T04(1:3,1),Rd(:,1))+cross(T04(1:3,2),Rd(:,2))+cross(T04(1:3,3),Rd(:,3)))/2;%Posiciones angulares 
    e = [v;w]; % Error
    
    switch caso
        case 1
            Jc = J(1:3,1:3);%Acotacion matriz jacobiana
            [m,n] = size(Jc);%Tamaño matriz jacobiana
            qp = pinv(Jc)*(k(1:3,1:3)*v);%Cinemática diferencial
            q(1:3) = q(1:3)+(qp*t);%Posiciones de las articulaciones
            u = abs(det(Jc));%Manipulabilidad
        case 2
            Jc = J(1:3,:);%Acotacion matriz jacobiana
            [m,n] = size(Jc);%Tamaño matriz jacobiana
            qp = pinv(Jc)*(k(1:3,1:3)*v);%Cinemática diferencial
            q = q+(qp*t);%Posiciones de las articulaciones 
            u = sqrt(det(Jc*Jc'));%Manipulabilidad
        case 3
            [m,n] = size(J);%Tamaño matriz jacobiana
            qp = pinv(J)*(k*e);%Cinemática diferencial
            q = q+(qp*t);%Posiciones de las articulaciones
            u = sqrt(det(J'*J));%Manipulabilidad
        otherwise
            return;
    end

    Q(:,i) = q;%Posiciones de las articulaciones
    time(i) = i*t;%Tiempo
    V(:,i) = v;%Velocidades de las articulaciones
    U(:,i) = u;%Manipulabilidad en cada iteracion
end

disp(['DOF redundantes=' num2str(n-m)])%Impresion en pantalla
if caso==1
    syms q1 q2 q3 q4 A1 A2
    qd = [q1 q2 q3 q4];%Vector de cordenadas generalizadas
    Jd = [ -A2*sin(qd(1)+qd(2))-A1*sin(qd(1)), -A2*sin(qd(1)+qd(2)), 0;
          A2*cos(qd(1)+qd(2))+A1*cos(qd(1)), A2*cos(qd(1)+qd(2)), 0;
          0, 0, -1];%Matriz jacobiana
    d = det(Jd);%Determinante
    
    disp('Singularidades:')%Impresion en pantalla
    disp(simplify(d))%Impresion en pantalla
    disp('theta2=0, theta2=pi')%Impresion en pantalla
    
end

% bot.plot(Q'+0.00001,'workspace',[-1 1 -1 1 -1 1])%Plot robot

figure
hold on; grid on;

subplot(3,1,1)
plot(time,Q)%Plot posiciones
title('Posiciones')

subplot(3,1,2)
plot(time,V)%Plot velocidades
title('Velocidades')

subplot(3,1,3)
plot(time,U)%Plot manipulabilidad
title('Manipulabilidad')