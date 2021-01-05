function [fx, fy] = f_atraccion(k_att, xg, yg, x, y)
    fx = -k_att * (x - xg);
    fy = -k_att * (y - yg); 