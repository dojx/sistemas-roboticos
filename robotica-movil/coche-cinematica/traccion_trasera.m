%Guzmán Magallón Gerardo
clear all;close all;clc

vs = -0.4;           %Velocidad lineal
w_alpha = 0.8;          %Velocidad angular
d = 0.3;
p = [0 0 0 0]';



t = 0.01;           %Paso de tiempo
s=5;               %Tiempo

p_plot = zeros(4,s/t); 
time = t:t:s;

for i=1:s/t
%    clf;
%    Dibujar_Coche(p,d)
%    pause(0.0001)
%    
   pp = [0 0 0 0]';
   pp(1) = vs*cos(p(3));        %Vel X
   pp(2) = vs*sin(p(3));        %Vel Y
   pp(3) = vs/d * tan(p(4));    %Vel Theta
   pp(4) = w_alpha;             %Vel Alpha
   
   p = p + pp*t;        
   p_plot(:,i) = p;
end

subplot(1,2,1);
plot(time,p_plot(1,:),'b','LineWidth',2);hold on;
plot(time,p_plot(2,:),'r','LineWidth',2);
plot(time,p_plot(3,:),'m','LineWidth',2);
plot(time,p_plot(4,:),'g','LineWidth',2);
legend('x','y','\theta','\alpha');title('Posiciones');

subplot(1,2,2);
plot(p_plot(1,:),p_plot(2,:),'g','LineWidth',2);hold on;title('Trayectoria');