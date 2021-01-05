% Actividad 8
clear all; close all; clc;

cilindrico;
%stanford;

%% 1. Manipulador Cilindrico de 6 DOF
function cilindrico()
  t = [0.6; 0.3; 0.8];%Vector de desplazamiento
  Rd=[1 0 0; 0 .7071  -.7071; 0 0.7071  0.7071]; %Matriz del actuador final

  % Matriz DH
  a = [0 0 0 0 0 0];
  alpha = [pi/2 -pi/2 0 -pi/2 pi/2 0];
  d = [0.35 0 0 0 0 0.1];
  theta = [0 0 0 0 0 0];
  offset = [0 -pi/2 0.35 0 0 0];

  tc = t - d(6)*(Rd*[0 0 1]');

  % Calculos de la 1ra parte (Esferico de 3 DOF)
  theta(1) = atan2(tc(2), tc(1));
  theta(2) = atan2(tc(3) - d(1), sqrt(tc(1)^2 + tc(2)^2));
  d(3) = sqrt(tc(1)^2 + tc(2)^2 + (tc(3) - d(1))^2) - offset(3);

  % Toolbox
  L1 = Revolute('a',a(1),'alpha',alpha(1),'d',d(1),'offset',offset(1));
  L2 = Revolute('a',a(2),'alpha',alpha(2),'d',d(2),'offset',offset(2));
  L3 = Prismatic('a',a(3),'alpha',alpha(3),'theta',theta(3),'offset',offset(3));

  bot = SerialLink([L1 L2 L3],'name','Esferico 3DOF');

  q = [theta(1) theta(2) d(3)];
  T_03 = fkine(bot, q);

  % Calculos para la esfera
  R_36 = T_03((1:3),(1:3))' * Rd;

  q(5) = atan2(sqrt(1-R_36(3,3)^2)', R_36(3,3));

  if q(5) >= 0
      q(4) = atan2(R_36(2,3),R_36(1,3));
      q(6) = atan2(R_36(3,2),-R_36(3,1));
  else
      q(4) = atan2(-R_36(2,3),-R_36(1,3));
      q(6) = atan2(-R_36(3,2),R_36(3,1));
  end

  % Toolbox
  L4 = Revolute('a',a(4),'alpha',alpha(4),'d',d(4),'offset',offset(4));
  L5 = Revolute('a',a(5),'alpha',alpha(5),'d',d(5),'offset',offset(5));
  L6 = Revolute('a',a(6),'alpha',alpha(6),'d',d(6),'offset',offset(6));

  bot2 = SerialLink([L4 L5 L6]);
  T_36 = fkine(bot2, [q(4) q(5) q(6)]);%Matriz homogenea de la esfera

  T_06 = T_03*T_36 %Matriz homogenea del sistema completo

  bot_completo = SerialLink([L1 L2 L3 L4 L5 L6],'name','Manipulador Cilíndrico');
  bot_completo.plot([q(1) q(2) q(3) q(4) q(5) q(6)]+0.0001,'workspace',[-1 1 -1 1 -1 1]);
end

%% 2. Manipulador Stanford de 6 DOF
function stanford()
  t = [0.6; 0.3; 0.6];%Vector de desplazamiento
  Rd=[1 0 0; 0 0.7071  -0.7071; 0 0.7071  0.7071]; %Matriz del actuador final

  % Matriz DH
  a = [0 0 0 0 0 0];
  alpha = [0 pi/2 0 -pi/2 pi/2 0];
  d = [0.35 0 0 0 0 0.1];
  theta = [0 0 0 0 0 0];
  offset = [pi/2 0.15 0.15 0 0 0];

  tc = t - d(6)*(Rd*[0 0 1]');

  % Calculos de la 1ra parte (Cilindrico de 3 DOF)
  theta(1) = atan2(tc(2), tc(1));
  d(2) = t(3) - d(1) - offset(2);
  d(3) = sqrt(tc(1)^2 + tc(2)^2) - offset(3);

  % Toolbox
  L1 = Revolute('a',a(1),'alpha',alpha(1),'d',d(1),'offset',offset(1));
  L2 = Prismatic('a',a(2),'alpha',alpha(2),'theta',theta(2),'offset',offset(2));
  L3 = Prismatic('a',a(3),'alpha',alpha(3),'theta',theta(3),'offset',offset(3));

  bot = SerialLink([L1 L2 L3],'name','Cilindrico 3DOF');

  q = [theta(1) d(2) d(3)];
  T_03 = fkine(bot, q);

  % Calculos para la esfera
  R_36 = T_03((1:3),(1:3))' * Rd;

  q(5) = atan2(sqrt(1-R_36(3,3)^2)', R_36(3,3));

  if q(5) >= 0
      q(4) = atan2(R_36(2,3),R_36(1,3));
      q(6) = atan2(R_36(3,2),-R_36(3,1));
  else
      q(4) = atan2(-R_36(2,3),-R_36(1,3));
      q(6) = atan2(-R_36(3,2),R_36(3,1));
  end

  % Toolbox
  L4 = Revolute('a',a(4),'alpha',alpha(4),'d',d(4),'offset',offset(4));
  L5 = Revolute('a',a(5),'alpha',alpha(5),'d',d(5),'offset',offset(5));
  L6 = Revolute('a',a(6),'alpha',alpha(6),'d',d(6),'offset',offset(6));

  bot2 = SerialLink([L4 L5 L6]);
  T_36 = fkine(bot2, [q(4) q(5) q(6)]);%Matriz homogenea de la esfera

  T_06 = T_03*T_36 %Matriz homogenea del sistema completo

  bot_completo = SerialLink([L1 L2 L3 L4 L5 L6],'name','Manipulador Stanford');
  bot_completo.plot([q(1) q(2) q(3) q(4) q(5) q(6)]+0.0001,'workspace',[-1 1 -1 1 -1 1]);
end