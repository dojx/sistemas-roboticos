%% Actividad 5
T_01 = @(tht1) [cos(tht1) 0 sin(tht1) 0; 
                sin(tht1) 0 -cos(tht1) 0; 
                0 1 0 0.5; 
                0 0 0 1];
            
T_12 = @(tht2) [cos(tht2) -sin(tht2) 0 0.4*cos(tht2); 
                sin(tht2) cos(tht2) 0 0.4*sin(tht2); 
                0 0 1 0; 
                0 0 0 1];
            
T_23 = @(tht3) [cos(tht3) 0 sin(tht3) 0; 
                sin(tht3) 0 -cos(tht3) 0; 
                0 1 0 0; 
                0 0 0 1];
            
T_34 = @(tht4) [cos(tht4) -sin(tht4) 0 0; 
                sin(tht4) cos(tht4) 0 0; 
                0 0 1 0.6; 
                0 0 0 1];
            
q = [0 pi/4 pi/4 0]';           
            
T_04 = T_01(q(1)) * T_12(q(2)) * T_23(q(3)) * T_34(q(4))

%% Grafica
L1 = Revolute('a',0,'alpha',pi/2,'d',0.5,'offset',0);
L2 = Revolute('a',0.4,'alpha',0,'d',0,'offset',0);
L3 = Revolute('a',0,'alpha',pi/2,'d',0,'offset',0);
L4 = Revolute('a',0,'alpha',0,'d',0.6,'offset',0);

bot = SerialLink([L1 L2 L3 L4],'name','Cilindrico');

q = [0 pi/4 pi/4 0];
bot.plot(q+0.0001,'workspace',[-1 1 -1 1 -1 1])