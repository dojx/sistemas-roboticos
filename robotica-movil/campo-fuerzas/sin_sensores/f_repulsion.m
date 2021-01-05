function [fx, fy] = f_repulsion(k_rep, phi, x0, y0, x, y)
    phi0 = sqrt((x - x0)^2 + (y - y0)^2);
    
    if phi0 <= phi
        fx = k_rep * ((1/phi0)-(1/phi))*(1/phi0^2)*((x - x0)/phi0);
        fy = k_rep * ((1/phi0)-(1/phi))*(1/phi0^2)*((y - y0)/phi0);
    else
        fx = 0;
        fy = 0;
    end