function [y, dy, ddy] = roadProfile(x, road)

switch road.type
    case 1
        % Straight
        y = 0;
        dy = 0;
        ddy = 0;

    case 2
        % Single Sine
        w = 2*pi/road.L;
        y   = road.A * sin(w*x);
        dy  = road.A * w * cos(w*x);
        ddy = -road.A * w^2 * sin(w*x);

    case 3
        % Dounble Sine
        w1 = 2*pi/road.L1;
        w2 = 2*pi/road.L2;
        y   = road.A1*sin(w1*x) + road.A2*sin(w2*x);
        dy  = road.A1*w1*cos(w1*x) + road.A2*w2*cos(w2*x);
        ddy = -road.A1*w1^2*sin(w1*x) - road.A2*w2^2*sin(w2*x);

    case 4
        % S-Type
        z1 = (x - road.x1)/road.w;
        z2 = (x - road.x2)/road.w;

        y = road.A*tanh(z1) - road.A*tanh(z2);
        dy = road.A*(sechLocal(z1)^2)/road.w ...
           - road.A*(sechLocal(z2)^2)/road.w;
        ddy = road.A*(-2*sechLocal(z1)^2*tanh(z1))/road.w^2 ...
            - road.A*(-2*sechLocal(z2)^2*tanh(z2))/road.w^2;

    otherwise
        warning('KnownRoadType %g', road.type);
        y = 0;
        dy = 0;
        ddy = 0;
end
end
