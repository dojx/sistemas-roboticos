close all; clear; clc;
syms p1 p2 q1 q2 q3

% Transformacion homogenea para una plataforma omnidireccional
T_wp = [cos(p2) -sin(p2) 0 p1*cos(p2);
        sin(p2) cos(p2) 0 p1*sin(p2);
        0 0 1 0;
        0 0 0 1];
% Transformacion de base al primer eslabon                       
T_01 = [cos(q1) 0 sin(q1) 0;
        sin(q1) 0 -cos(q1) 0;
        0 1 0 0.35;
        0 0 0 1];
% Transformacion de primer eslabon a segundo eslabon           
T_12 = [cos(q2) -sin(q2) 0 0.3*cos(q2);
         sin(q2) cos(q2) 0 0.3*sin(q2);
         0 0 1 0;
         0 0 0 1];
% Transformacion de segundo eslabon a tercer eslabon            
T_23 = [cos(q3) -sin(q3) 0 0.25*cos(q3);
        sin(q3) cos(q3) 0 0.25*sin(q3);
        0 0 1 0;
        0 0 0 1];        
% Transformacion de base a actuador final           
T_be = T_01 * T_12 * T_23;
                    
% Transformacion de la base movil a base del manipulador
T_pb = [1 0 0 0.25;
        0 1 0 0;
        0 0 1 0.25;
        0 0 0 1];

% Cinematica directa para el actuador final con respecto al eje global
T_we = T_wp* T_pb * T_be;

vt = T_we(1:3, 4); % Vector de traslacion

% Matriz Jacobiana
J = [diff(vt(1), p1) diff(vt(1), p2) diff(vt(1), q1) diff(vt(1), q2) diff(vt(1), q3);
    diff(vt(2), p1) diff(vt(2), p2) diff(vt(2), q1) diff(vt(2), q2) diff(vt(2), q3);
    diff(vt(3), p1) diff(vt(3), p2) diff(vt(3), q1) diff(vt(3), q2) diff(vt(3), q3)];

J = simplify(J);