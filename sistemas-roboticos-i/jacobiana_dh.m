% Actividad 10
clear all; close all; clc;

n = 1; % 1 = Antropomorfico, 2 = Cilindrico, 3 = Esferico, 4 = Planar
[J_v, J_w, J_f] = act10(n)

function [J_v, J_w, J_f] = act10(n)
  syms a1 a2 a3 d1 d2 d3 tht1 tht2 tht3
  
  % Tablas DH
  % 1. Manipulador antropomorfico de 3 DOF
  if n == 1
    a = [0 0.30 0.25];
    alpha = [pi/2 0 0];
    d = [0.35 0 0];
    theta = [tht1 tht2 tht3];
    q = [tht1 tht2 tht3];
  end
  % 2. Manipulador cilindrico de 3 DOF
  if n == 2
    a = [0 0 0];
    alpha = [0 pi/2 0];
    d = [0.35 d2 d3];
    theta = [tht1 0 0];
    q = [tht1 d2 d3];
  end
  % 3. Manipulador esferico de 3 DOF
  if n == 3
    a = [0 0 0];
    alpha = [pi/2 -pi/2 0];
    d = [0.35 0 d3];
    theta = [tht1 tht2 0];
    q = [tht1 tht2 d3];
  end
  % 4. Manipulador planar de 3 DOF
  if n == 4
    a = [0.35 0.35 0.25];
    alpha = [0 0 0];
    d = [0 0 0];
    theta = [tht1 tht2 tht3];
    q = [tht1 tht2 tht3];
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
  T_01 = T(a(1), alpha(1), d(1), theta(1))
  T_12 = T(a(2), alpha(2), d(2), theta(2));
  T_23 = T(a(3), alpha(3), d(3), theta(3));
  T_02 = simplify(T_01 * T_12);
  T_03 = simplify(T_02 * T_23);

  % Posicion de actuador final
  t0 = [0; 0; 0];
  t1 = T_01(1:3, 4);
  t2 = T_02(1:3, 4);
  t3 = T_03(1:3, 4);
  
  % Checar si es rotacional o prismatico
  p = zeros(3, 1);
  for i = 1:3
    if(q(i) == tht1 || q(i) == tht2 || q(i) == tht3)
      p(i) = 1;
    end
  end
  
  % Ejes Z
  z = [[0; 0; 1] T_01(1:3, 3) T_02(1:3, 3)];

  % Lineal
  J_v = [simplify(cross(z(:, 1), (t3 - t0))) simplify(cross(z(:, 2), (t3 - t1))) simplify(cross(z(:, 3), (t3 - t2)))];

  % Angular
  J_w = [p(1)*z(:, 1) p(2)*z(:, 2) p(3)*z(:, 3)];

  J_f = [J_v; J_w];
end