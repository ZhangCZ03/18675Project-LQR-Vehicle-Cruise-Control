function [Xg, Yg] = getCarShape(xc, yc, psi, L, W)

X = [ L/2,  L/2, -L/2, -L/2];
Y = [ W/2, -W/2, -W/2,  W/2];

R = [cos(psi), -sin(psi);
     sin(psi),  cos(psi)];

P = R * [X; Y];
Xg = P(1,:) + xc;
Yg = P(2,:) + yc;
end
