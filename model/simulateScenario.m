function out = simulateScenario(P, road, dist, K)
%SIMULATESCENARIO 用非线性自行车模型运行一次闭环仿真。

if nargin < 4 || isempty(K)
    K = buildLQRController(P);
end

lane_bound = P.lane_width/2;
N = round(P.T/P.dt);
g = 9.81;

% 初始状态
vx   = P.vx0;
vy   = P.vy0;
psi  = P.psi0;
r    = P.r0;
xpos = P.x0;
ypos = P.y0;

a_prev = 0;
ay_prev = 0;

% 数据记录
time_hist   = zeros(N,1);
x_hist      = zeros(N,1);
y_hist      = zeros(N,1);
vx_hist     = zeros(N,1);
vy_hist     = zeros(N,1);
psi_hist    = zeros(N,1);
r_hist      = zeros(N,1);
ey_hist     = zeros(N,1);
epsi_hist   = zeros(N,1);
a_hist      = zeros(N,1);
delta_hist  = zeros(N,1);
yref_hist   = zeros(N,1);
psiref_hist = zeros(N,1);
kappa_hist  = zeros(N,1);
ay_hist     = zeros(N,1);
ax_hist     = zeros(N,1);
jerk_hist   = zeros(N,1);
vxref_hist  = zeros(N,1);
grade_hist  = zeros(N,1);

for k = 1:N
    t = (k-1)*P.dt;

    vx_ref_now = P.vx_ref;
    if dist.useSpeedStep && t >= dist.stepTime
        vx_ref_now = dist.vxRef2;
    end

    [y_ref, dy_dx, ddy_dx2] = roadProfile(xpos, road);

    if dist.useCurvatureBoost && xpos >= dist.curvBoostStart && xpos <= dist.curvBoostEnd
        dy_dx   = dist.curvBoostScale * dy_dx;
        ddy_dx2 = dist.curvBoostScale * ddy_dx2;
    end

    psi_ref   = atan(dy_dx);
    kappa_ref = ddy_dx2 / (1 + dy_dx^2)^(3/2);

    % Current road grade
    theta_grade = roadGradeProfile(xpos, dist);

   
    m_now  = P.m  * dist.massScale;
    Iz_now = P.Iz * dist.massScale;
    Cf_now = P.Cf * dist.CfScale;
    Cr_now = P.Cr * dist.CrScale;

    % Tracking error
    ev   = vx - vx_ref_now;
    ey   = ypos - y_ref;
    epsi = wrapToPiLocal(psi - psi_ref);

    x_state = [ev; ey; epsi; vy; r];

    % LQR feedback
    u_fb = -K * x_state;
    a_cmd = u_fb(1);

    % Curvature feedforward
    delta_ff = atan((P.lf + P.lr) * kappa_ref);
    delta_cmd = u_fb(2) + delta_ff;

    % saturation
    a_cmd     = min(max(a_cmd, P.a_min), P.a_max);
    delta_cmd = min(max(delta_cmd, -P.delta_max), P.delta_max);

    % non-linear bicycle model
    vx_safe = max(vx, 0.5);

    alpha_f = delta_cmd - atan2(vy + P.lf*r, vx_safe);
    alpha_r = -atan2(vy - P.lr*r, vx_safe);

    Fyf = 2 * Cf_now * alpha_f;
    Fyr = 2 * Cr_now * alpha_r;

    vx_dot   = a_cmd - g * sin(theta_grade);
    vy_dot   = (Fyf + Fyr)/m_now - vx*r;
    r_dot    = (P.lf*Fyf - P.lr*Fyr)/Iz_now;
    psi_dot  = r;
    xpos_dot = vx*cos(psi) - vy*sin(psi);
    ypos_dot = vx*sin(psi) + vy*cos(psi);

    % Comfort
    ax_now = vx_dot;
    ay_now = vy_dot + vx*r;
    jerk_x_now = (a_cmd - a_prev) / P.dt;
    jerk_y_now = (ay_now - ay_prev) / P.dt;
    total_jerk_now = sqrt(jerk_x_now^2 + jerk_y_now^2);

    vx   = vx   + vx_dot*P.dt;
    vy   = vy   + vy_dot*P.dt;
    r    = r    + r_dot*P.dt;
    psi  = psi  + psi_dot*P.dt;
    xpos = xpos + xpos_dot*P.dt;
    ypos = ypos + ypos_dot*P.dt;


    time_hist(k)   = t;
    x_hist(k)      = xpos;
    y_hist(k)      = ypos;
    vx_hist(k)     = vx;
    vy_hist(k)     = vy;
    psi_hist(k)    = psi;
    r_hist(k)      = r;
    ey_hist(k)     = ey;
    epsi_hist(k)   = epsi;
    a_hist(k)      = a_cmd;
    delta_hist(k)  = delta_cmd;
    yref_hist(k)   = y_ref;
    psiref_hist(k) = psi_ref;
    kappa_hist(k)  = kappa_ref;
    ax_hist(k)     = ax_now;
    ay_hist(k)     = ay_now;
    jerk_hist(k)   = total_jerk_now;
    vxref_hist(k)  = vx_ref_now;
    grade_hist(k)  = theta_grade;

    a_prev = a_cmd;
    ay_prev = ay_now;
end

out.time_hist   = time_hist;
out.x_hist      = x_hist;
out.y_hist      = y_hist;
out.vx_hist     = vx_hist;
out.vy_hist     = vy_hist;
out.psi_hist    = psi_hist;
out.r_hist      = r_hist;
out.ey_hist     = ey_hist;
out.epsi_hist   = epsi_hist;
out.a_hist      = a_hist;
out.delta_hist  = delta_hist;
out.yref_hist   = yref_hist;
out.psiref_hist = psiref_hist;
out.kappa_hist  = kappa_hist;
out.ax_hist     = ax_hist;
out.ay_hist     = ay_hist;
out.jerk_hist   = jerk_hist;
out.vxref_hist  = vxref_hist;
out.grade_hist  = grade_hist;
out.metrics = computeMetrics(time_hist, vx_hist, vxref_hist, ey_hist, delta_hist, ...
                             ax_hist, ay_hist, jerk_hist, lane_bound);
end
