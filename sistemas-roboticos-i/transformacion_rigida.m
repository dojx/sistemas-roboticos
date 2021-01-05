%% Actividad 4
clc; clear all; close all;

%% Matrices Homogeneas
TAB = [0 1 0 2; -1 0 0 2; 0 0 1 2; 0 0 0 1];
TBC = [1 0 0 2; 0 0 -1 2; 0 1 0 -1; 0 0 0 1];
TAD = [0 -1 0 6; 1 0 0 -6; 0 0 1 3; 0 0 0 1];
TAC = TAB * TBC
TCD = inv(TAC) * TAD % inv(TAC) = TCA

%% Puntos Homogeneos
pB = [0; -2; 0; 1];
pA = TAB * pB
pC = inv(TAC) * pA  
pD = inv(TCD) * pC % inv(TCD) = TDC

figure
Dibujar_Sistema_Referencia_3D(eye(4,4), 'a') % Dibujar sistema 'a' en el origin 
Dibujar_Sistema_Referencia_3D(TAB, 'b') % Dibujar sistema 'b'
Dibujar_Sistema_Referencia_3D(TAC, 'c') % Dibujar sistema 'c'
Dibujar_Sistema_Referencia_3D(TAD, 'd') % Dibujar sistema 'd'
Dibujar_Punto_3D(pA) % Dibujar pA en el sistema 'a'

function Dibujar_Punto_3D(at) % Dibujar un punto con respecto al origin
  hold on
  grid on
  axis([-10 10 -10 10 0 10])
  xlabel('x')
  ylabel('y')
  zlabel('z')

  plot3(at(1), at(2), at(3), 'bx', 'MarkerSize', 10, 'LineWidth',2)
  plot3(at(1), at(2), at(3), 'ro', 'MarkerSize', 10,  'LineWidth',2)
  view([-45,30])
end

function Dibujar_Sistema_Referencia_3D(aTb, s) % Dibujar sistemas de referencia y nombrarlos
  hold on
  grid on
  axis([-10 10 -10 10 0 10])
  xlabel('x')
  ylabel('y')
  zlabel('z')

  apx = aTb*[2 0 0 1]';
  apy = aTb*[0 2 0 1]';
  apz = aTb*[0 0 2 1]';    
  atb = aTb(1:3, 4);   
   
  line([atb(1) apx(1)], [atb(2) apx(2)], [atb(3) apx(3)], 'color', [1 0 0], 'LineWidth', 3)
  line([atb(1) apy(1)], [atb(2) apy(2)], [atb(3) apy(3)], 'color', [0 1 0], 'LineWidth', 3)
  line([atb(1) apz(1)], [atb(2) apz(2)], [atb(3) apz(3)], 'color', [0 0 1], 'LineWidth', 3)    
  plot3(atb(1), atb(2), atb(3), 'k.', 'MarkerSize', 25)
  text(atb(1), atb(2), atb(3)-0.75, s, 'Color', 'blue', 'FontSize', 15)
  view([-45, 30])
end