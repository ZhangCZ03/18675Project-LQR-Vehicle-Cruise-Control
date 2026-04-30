function [P, road, dist] = defaultParams()

%Parameter
P.m  = 1500;        % kg
P.Iz = 3000;        % kg*m^2
P.lf = 1.2;         % m
P.lr = 1.6;         % m
P.Cf = 8e4;         % N/rad
P.Cr = 8e4;         % N/rad

P.vx_ref     = 20;      % m/s
P.lane_width = 3.5;     % m
P.dt         = 0.01;    % s
P.T          = 16;      % s

P.vx0   = 10;
P.vy0   = 0;
P.psi0  = 0;      
P.r0    = 0;
P.x0    = 0;    
P.y0    = 0;      

P.start_s      = 0;  
P.start_offset = 0;   

%LQR Weight
P.qv    = 60;
P.qy    = 1600;
P.qpsi  = 450;
P.qvy   = 8;
P.qr    = 12;
P.ra    = 1;
P.rdel  = 20;

%Saturation
P.a_max     = 2.5;
P.a_min     = -3.0;
P.delta_max = deg2rad(39);

%Road Type
road.type = 3;   
road.A1   = 1.2;
road.L1   = 70;
road.A2   = 0.5;
road.L2   = 25;
road.A    = 1.5;
road.L    = 80;
road.x1   = 35;
road.x2   = 90;
road.w    = 12;

%Disturbance
dist.gradeType   = 0;             
dist.gradeValue  = deg2rad(4);    % slope
dist.gradeStart  = 40;            % start point x
dist.gradeEnd    = 90;            % end point x
dist.gradeAmp    = deg2rad(3);    % Sine grade amplitude
dist.gradeWave   = 60;            % Sine grade wavelength

dist.massScale = 1.0;
dist.CfScale   = 1.0;
dist.CrScale   = 1.0;

% Speed for step response
dist.useSpeedStep = false;
dist.stepTime     = 4.0;
dist.vxRef2       = 25;

% Curvature step-change
dist.useCurvatureBoost = false;
dist.curvBoostStart    = 50;
dist.curvBoostEnd      = 90;
dist.curvBoostScale    = 1.3;
end
