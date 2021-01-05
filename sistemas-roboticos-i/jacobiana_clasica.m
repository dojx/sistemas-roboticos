% Actividad 9
clear all; close all; clc;

n = 1; % 1 = Planar, 2 = Antropomorfico, 3 = Cilindrico, 4 = Esferico
[J_v, J_w, J_f] = act9(n)

function [J_v, J_w, J_f] = act9(n)
  syms a1 a2 a3 d1 d2 d3 tht1 tht2 tht3 tht4 a4 d4
  
  % Tablas DH
  % 1. Manipulador planar de 3 DOF
  if n == 1
    a = [a1 a2 a3 a4];
    alpha = [0 0 0 0];
    d = [0 0 0 0];
    theta = [tht1 tht2 tht3 tht4];
    q = [tht1 tht2 tht3 tht4];
  end
  % 2. Manipulador antropomorfico de 3 DOF
  if n == 2
    a = [0 a2 a3];
    alpha = [pi/2 0 0];
    d = [d1 0 0];
    theta = [tht1 tht2 tht3];
    q = [tht1 tht2 tht3];
  end
  % 3. Manipulador cilindrico de 3 DOF
  if n == 3
    a = [0 0 0];
    alpha = [0 pi/2 0];
    d = [d1 d2 d3];
    theta = [tht1 0 0];
    q = [tht1 d2 d3];
  end
  % 4. Manipulador esferico de 3 DOF
  if n == 4
    a = [0 0 0];
    alpha = [pi/2 -pi/2 0];
    d = [d1 0 d3];
    theta = [tht1 tht2 0];
    q = [tht1 tht2 d3];
  end

  % Calcular Jacobiana
  [J_v, J_w, J_f] = jacobiana(a, alpha, d, theta, q);
end
                           
function [J_v, J_w, J_f] = jacobiana(a, alpha, d, theta, q)
  syms a1 a2 a3 d1 d2 d3 tht1 tht2 tht3
  % Formula para matrices de transformacion
  T = @(a, alpha, d, theta) [cos(theta) -sin(theta)*floor(cos(alpha)) sin(theta)*sin(alpha) a*cos(theta);
                             sin(theta) cos(theta)*floor(cos(alpha)) -cos(theta)*sin(alpha) a*sin(theta);
                             0 sin(alpha) floor(cos(alpha)) d;
                             0 0 0 1];

  % Calcular matrices de transformacion
  T_01 = T(a(1), alpha(1), d(1), theta(1));
  T_12 = T(a(2), alpha(2), d(2), theta(2));
  T_23 = T(a(3), alpha(3), d(3), theta(3));
  T_34 =T(a(4),alpha(4),d(4),theta(3));
  T_02 = simplify(T_01 * T_12);
  T_03 = simplify(T_02 * T_23);
  T_04 = simplify(T_03 * T_34);

  % Posicion de actuador final
  tx = T_04(1, 4);
  ty = T_04(2, 4);
  tz = T_04(3, 4);
  
  % Checar si es rotacional o prismatico
  p = zeros(4, 1);
  for i = 1:3
    if(q(i) == tht1 || q(i) == tht2 || q(i) == tht3 q(i) == tht4)
      p(i) = 1;
    end
  end
  
  % Ejes Z
  z = [[0; 0; 1] T_01(1:3, 3) T_02(1:3, 3) T_03(1:3, 3)];

  % Lineal
  J_v = [diff(tx, q(1)) diff(tx, q(2)) diff(tx, q(3));
         diff(ty, q(1)) diff(ty, q(2)) diff(ty, q(3));
         diff(tz, q(1)) diff(tz, q(2)) diff(tz, q(3))];

  % Angular
  J_w = [p(1)*z(:, 1) p(2)*z(:, 2) p(3)*z(:, 3)];

  J_f = [J_v; J_w];
end