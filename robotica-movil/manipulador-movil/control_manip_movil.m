
T = @(a,alpha,d,theta) [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
    0 sin(alpha) cos(alpha) d; 0 0 0 1];


wTp = @(p) [[rotz(p(3))  [p(1:2);0]];0 0 0 1];
pTb = [eye(3) [0.25;0;0.25];0 0 0 1];
T01 = @(theta) T(0,pi/2,0.35,theta);
T12 = @(theta) T(0.3,0,0,theta);
T23 = @(theta) T(0.25,0,0,theta);

bTe = @(q) T01(q(1))*T12(q(2))*T23(q(3));

q_arm = [0 pi/4 -pi/4]';
p_base = [0 0 0]';
q = [p_base;q_arm];

dt = 0.01;
xd = [1 1 0.5]'; %deseao
N = 1000;

K = eye(1);

xplot = zeros(3,N);
qplot = zeros(3,N);

for i=1:N
   wTe = wTp(q(1:3))* pTb * bTe(q(4:6));

   J = jacob(q);
   J_inv = pinv(J);
   
   qp = J_inv*K*(xd - wTe(1:3,4));
   
   q = q + qp*dt;
   
   xplot(1,i) = xd(1);
   qplot(1,i) = wTe(1,4);
   
   K*(xd - wTe(1:3,4))
end

plot(xplot(1,:));hold on;plot(qplot(1,:));



function J = jacob(q)
x_b = q(1);
y_b = q(2);
theta_b = q(3);
theta1 = q(4);
theta2 = q(5);
theta3 = q(6);



J = [[1, 0, (sin(theta1 + theta_b)*sin(theta2)*sin(theta3))/4 - (3*sin(theta1 + theta_b)*cos(theta2))/10 - (sin(theta1 + theta_b)*cos(theta2)*cos(theta3))/4 - sin(theta_b)/4, -(sin(theta1 + theta_b)*(5*cos(theta2 + theta3) + 6*cos(theta2)))/20, -(sin(theta1 + theta_b)*(5*cos(theta2 + theta3) + 6*cos(theta2)))/20, -(cos(theta1 + theta_b)*sin(theta2 + theta3))/4];
[0, 1, cos(theta_b)/4 + (3*cos(theta1 + theta_b)*cos(theta2))/10 + (cos(theta1 + theta_b)*cos(theta2)*cos(theta3))/4 - (cos(theta1 + theta_b)*sin(theta2)*sin(theta3))/4,  (cos(theta1 + theta_b)*(5*cos(theta2 + theta3) + 6*cos(theta2)))/20,  (cos(theta1 + theta_b)*(5*cos(theta2 + theta3) + 6*cos(theta2)))/20, -(sin(theta2 + theta3)*sin(theta1 + theta_b))/4];
[0, 0,                                                                                                                                                                 0,                                                                    0,                                                                    0,                          cos(theta2 + theta3)/4]];
end