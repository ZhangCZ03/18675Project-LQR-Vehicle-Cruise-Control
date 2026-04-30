function K = buildLQRController(P)

if P.vx_ref <= 0
    error('P.vx_ref must be greater than 0');
end

if exist('lqr', 'file') ~= 2
    error(['LQR not found']);
end

V = P.vx_ref;

A = [ 0,                          0,                 0,                              0,                                0;
      0,                          0,                 V,                              1,                                0;
      0,                          0,                 0,                              0,                                1;
      0,                          0,                 0,  -(2*P.Cf+2*P.Cr)/(P.m*V),  -(V + (2*P.Cf*P.lf-2*P.Cr*P.lr)/(P.m*V));
      0,                          0,                 0,  -(2*P.Cf*P.lf-2*P.Cr*P.lr)/(P.Iz*V), ...
                                                      -(2*P.Cf*P.lf^2+2*P.Cr*P.lr^2)/(P.Iz*V)];

B = [1,          0;
     0,          0;
     0,          0;
     0,      2*P.Cf/P.m;
     0,  2*P.Cf*P.lf/P.Iz];

Q = diag([P.qv, P.qy, P.qpsi, P.qvy, P.qr]);
R = diag([P.ra, P.rdel]);

K = lqr(A, B, Q, R);
end
