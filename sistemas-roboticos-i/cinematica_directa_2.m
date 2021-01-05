close all; clear all; clc;

L1 = Revolute('a',0,'alpha',pi/2,'d',0.5,'offset',0);
L2 = Revolute('a',.4,'alpha',0,'d',0,'offset',0);
L3 = Revolute('a',0,'alpha',pi/2,'d',0,'offset',0);
L4 = Revolute('a',0,'alpha',0,'d',.6,'offset',0);

bot = SerialLink([L1 L2 L3 L4 ],'name','Act 6');

q=[0,pi/4,pi/4,0]; 

% Matrices de transformacion 
T01=bot.A(1,q);
T12=bot.A(2,q);
T23=bot.A(3,q);
T34=bot.A(4,q);

% Matriz final multiplicando cada una de las matrices
T04=T01*T12*T23*T34

% Matriz final utilizando el tool box
T04 = fkine(bot, q)
bot.plot(q+0.0001,'workspace',[-1 1 -1 1 -1 1]);