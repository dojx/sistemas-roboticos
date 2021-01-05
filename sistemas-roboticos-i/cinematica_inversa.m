clear all; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1. Manipulador planar de 2 DOF
% Soluciones:
% tht = [0.5003 0.6897]        
% tht = [1.0705 -0.6897]

tx = 0.4;
ty = 0.4;

a = [0.35 0.25]';
tht = [0 0];

tht(2) = acos((tx^2+ty^2-a(1)^2-a(2)^2)/(2*a(1)*a(2))); % +-
tht(1) = atan2(ty,tx)-asin((a(2)*sin(tht(2)))/sqrt(tx^2+ty^2));

% Toolbox
L1 = Revolute('a',a(1),'alpha',0,'d',0,'offset',0);
L2 = Revolute('a',a(2),'alpha',0,'d',0,'offset',0);
bot = SerialLink([L1 L2],'name','Planar 2DOF');
q = tht;
bot.plot(q+0.0001,'workspace',[-1 1 -1 1 -1 1]);
T01 = bot.A(1, q);
T12 = bot.A(2, q);
T02 = fkine(bot, q)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% 2. Manipulador antropomorfico de 3 DOF
% % Soluciones:
% % tht = [0.5880   -0.7553    1.7214]        
% % tht = [0.5880    0.7553   -1.7214]
% tx = 0.3;
% ty = 0.2;
% tz = 0.35;
% 
% a = [0 0.3 0.25]';
% alpha = [pi/2 0 0]';
% d = [0.35 0 0]';
% tht = [0 0 0];
% 
% tht(1) = atan2(ty, tx);
% tht(3) = acos((tx^2 + ty^2 + (tz - d(1))^2 - a(2)^2 - a(3)^2)/(2*a(2)*a(3))); % +-
% tht(2) = atan2(tz - d(1), sqrt(tx^2 + ty^2)) - asin((a(3)*sin(tht(3)))/sqrt(tx^2 + ty^2 + (tz - d(1))^2));
% 
% % Toolbox
% L1 = Revolute('a',a(1),'alpha',alpha(1),'d',d(1),'offset',0);
% L2 = Revolute('a',a(2),'alpha',alpha(2),'d',d(2),'offset',0);
% L3 = Revolute('a',a(3),'alpha',alpha(3),'d',d(3),'offset',0);
% bot = SerialLink([L1 L2 L3],'name','Antropomorfico 3DOF');
% q = tht;
% bot.plot(q+0.0001,'workspace',[-1 1 -1 1 -1 1]);
% T01 = bot.A(1, q);
% T12 = bot.A(2, q);
% T23 = bot.A(3, q);
% T03 = fkine(bot, q)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% 3. Cilindrico de 3 DOF
% % Soluciones:
% % tht1 = [0.4636]   d2 = [0.3000]   d3 = [0.4090]     
% tx = 0.5;
% ty = 0.25;
% tz = 0.8;
% 
% a = [0 0 0]';
% alpha = [0 pi/2 0]';
% d = [0.35 0 0]';
% tht = [0 0 0];
% offset = [pi/2 0.15 0.15]';
% 
% tht(1) = atan2(ty, tx);
% d(2) = tz - d(1) - offset(2);
% d(3) = sqrt(tx^2 + ty^2) - offset(3);
% 
% % Toolbox
% L1 = Revolute('a',a(1),'alpha',alpha(1),'d',d(1),'offset',offset(1));
% L2 = Prismatic('a',a(2),'alpha',alpha(2),'theta',0,'offset',offset(2));
% L3 = Prismatic('a',a(3),'alpha',alpha(3),'theta',0,'offset',offset(3));
% bot = SerialLink([L1 L2 L3],'name','Cilindrico 3DOF');
% q = [tht(1) d(2) d(3)];
% bot.plot(q+0.0001,'workspace',[-1 1 -1 1 -1 1]);
% T01 = bot.A(1, q);
% T12 = bot.A(2, q);
% T23 = bot.A(3, q);
% T03 = fkine(bot, q)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% 4. Manipulador esferico de 3 DOF
% % Soluciones:
% % tht1 = [0.4636]   tht2 = [0.2622]   d3 = [0.2288]
% tx = 0.5;
% ty = 0.25;
% tz = 0.5;
% 
% a = [0 0 0];
% alpha = [pi/2 -pi/2 0];
% d = [0.35 0 0];
% tht = [0 0 0];
% offset = [0 -pi/2 0.35];
% 
% tht(1) = atan2(ty, tx);
% tht(2) = atan2(tz - d(1), sqrt(tx^2 + ty^2));
% d(3) = sqrt(tx^2 + ty^2 + (tz - d(1))^2) - offset(3);
% 
% % Toolbox
% L1 = Revolute('a',a(1),'alpha',alpha(1),'d',d(1),'offset',offset(1));
% L2 = Revolute('a',a(2),'alpha',alpha(2),'d',d(2),'offset',offset(2));
% L3 = Prismatic('a',a(3),'alpha',alpha(3),'theta',tht(3),'offset',offset(3));
% bot = SerialLink([L1 L2 L3],'name','Esferico 3DOF');
% q = [tht(1) tht(2) d(3)];
% bot.plot(q+0.0001,'workspace',[-1 1 -1 1 -1 1]);
% T01 = bot.A(1, q);
% T12 = bot.A(2, q);
% T23 = bot.A(3, q);
% T03 = fkine(bot, q)