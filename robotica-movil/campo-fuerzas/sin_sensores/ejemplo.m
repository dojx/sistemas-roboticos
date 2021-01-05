close all;
clear; clc;

xg = 5;
yg = 5;

x0_1 = 2;
y0_1 = 2.5;

x0_2 = 1.5;
y0_2 = 1.0;

x = 0;
y = 0;

k_att = 1;
k_rep = 5;
phi = 0.5;

N = 500;
h = 0.05;

figure
axis([-2 6 -2 6])
grid on
hold on
xlabel('x')
xlabel('y')

for i = 1 : N
    [fa_x, fa_y] = f_atraccion(k_att, xg, yg, x, y);
    [fr_x1, fr_y1] = f_repulsion(k_rep, phi, x0_1, y0_1, x, y);
    [fr_x2, fr_y2] = f_repulsion(k_rep, phi, x0_2, y0_2, x, y);
    
    fx = fa_x + fr_x1 + fr_x2;
    fy = fa_y + fr_y1 + fr_y2;
    
    n_fx = fx / sqrt(fx^2 + fy^2);
    n_fy = fy / sqrt(fx^2 + fy^2);
    
    x = x + h*n_fx;
    y = y + h*n_fy;
    
    plot(x, y, 'bo', 'MarkerSize', 10)
    plot(x0_1, y0_1, 'ro', 'MarkerSize', 10)
    plot(x0_2, y0_2, 'ro', 'MarkerSize', 10)
    plot(xg, yg, 'go', 'MarkerSize', 10)
    
    pause(0.01)
end