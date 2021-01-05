% Actividad 11
close all; clear all; clc;
syms d1 d2 d3 tht1 tht2 tht3

n = 3; % 

if n == 1
  % 1. Manipulador antropomorfico de 3 DOF
  a = [0 0.3 0.25];
  alpha = [pi/2 0 0];
  d = [0.35 0 0];
  theta = [tht1 tht2 tht3];
  td = [0.25 0.25 0.5]';
  q0 = [0.1 0.1 0.1]';
end

if n == 2
  % 2. Manipulador cilindrico de 3 DOF
  a = [0 0 0];
  alpha = [0 pi/2 0]';
  d = [0.35 d2 d3]';
  theta = [tht1 0 0]';
  td = [0.5 0.25 0.8]';
  q0 = [pi/4 pi/4 pi/4]';
end

if n == 3
  % 3. Manipulador esferico de 3 DOF
  a = [0 0 0]';
  alpha = [pi/2 -pi/2 0]';
  d = [0.35 0 d3]';
  theta = [tht1 tht2 0]';
  td = [0.5 0.25 0.5]';
  q0 = [0.1 0.1 0.1]';
end

if n == 4
  % 4. Manipulador planar de 3 DOF
  a = [0.35 0.35 0.25];
  alpha = [0 0 0]';
  d = [0 0 0]';
  theta = [tht1 tht2 tht3];
  td = [0.6 0.5 0]';
  q0 = [0.1 0.1 0.1]';
end

[q, e] = act11(a, alpha, d, theta, td, q0, n)

function [q0, e] = act11(a, alpha, d, theta, td, q0, n)
  syms d1 d2 d3 tht1 tht2 tht3
  t = 0.1;
  K = eye(3);

  % Formula para matrices de transformacion
  T = @(a, alpha, d, theta) [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
                                        sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
                                        0 sin(alpha) cos(alpha) d;
                                        0 0 0 1];
  e = [1 1 1]';

  % Figuras
  fig2 = figure('Name', 'Velocidad', 'Position', [650 300 600 500]);
  fig3 = figure('Name', 'Posicion', 'Position', [1300 300 600 500]);
  ax1 = axes(fig2); ax2 = axes(fig3); 
  hold(ax1, 'on'); hold(ax2, 'on');

  for i = t:t:1
    % Matrices de transformacion dependiendo del manipulador
    if n == 1
      T_01 = T(a(1), alpha(1), d(1), q0(1));
      T_12 = T(a(2), alpha(2), d(2), q0(2));
      T_23 = T(a(3), alpha(3), d(3), q0(3));
    end
    if n == 2
      T_01 = T(a(1), alpha(1), d(1), q0(1));
      T_12 = T(a(2), alpha(2), q0(2), theta(2));
      T_23 = T(a(3), alpha(3), q0(3), theta(3));
    end
    if n == 3
      T_01 = T(a(1), alpha(1), d(1), q0(1));
      T_12 = T(a(2), alpha(2), d(2), q0(2));
      T_23 = T(a(3), alpha(3), q0(3), theta(3));
    end
    if n == 4
      T_01 = T(a(1), alpha(1), d(1), q0(1));
      T_12 = T(a(2), alpha(2), d(2), q0(2));
      T_23 = T(a(3), alpha(3), d(3), q0(3));
    end
    T_02 = T_01 * T_12;
    T_03 = T_02 * T_23;

    % Velocidad lineal
    e = td - T_03(1:3, 4);

    % Posicion de actuador 
    t0 = [0; 0; 0];
    t1 = T_01(1:3, 4);
    t2 = T_02(1:3, 4);
    t3 = T_03(1:3, 4);

    % Ejes Z
    z = [[0; 0; 1] T_01(1:3, 3) T_02(1:3, 3)];
    % Jacobiano
    if n == 1 || n == 4
      J_v = [cross(z(:, 1), (t3 - t0)) cross(z(:, 2), (t3 - t1)) cross(z(:, 3), (t3 - t2))];
    end
    if n == 2
      J_v = [cross(z(:, 1), (t3 - t0)) z(:, 2) z(:, 3)];
    end
    if n == 3
      J_v = [cross(z(:, 1), (t3 - t0)) cross(z(:, 2), (t3 - t1)) z(:, 3)];
    end
    qp = pinv(J_v)*K*e; % Q punto
    q0 = q0 + qp*t; % Posicion
    % Graficar
    plot(ax1, i, e, 'r.');
    plot(ax2, i, q0, 'r.');
  end
end




