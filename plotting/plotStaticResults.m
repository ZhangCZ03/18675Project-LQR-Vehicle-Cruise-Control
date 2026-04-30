function plotStaticResults(out, P, road)

lane_bound = P.lane_width/2;
[x_road, y_road, y_up, y_dn] = generateRoadForPlot(out.x_hist, road, lane_bound);

figure('Color','w','Position',[100 80 1100 700]);

subplot(2,2,1);
plot(x_road, y_up, '--', 'LineWidth', 1.0); hold on;
plot(x_road, y_dn, '--', 'LineWidth', 1.0);
plot(x_road, y_road, '-.', 'LineWidth', 1.0);
plot(out.x_hist, out.y_hist, 'LineWidth', 2);
grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)');
title('Vehicle Trajectory and Road');
legend('Upper boundary','Lower boundary','Road centerline','Vehicle trajectory');

subplot(2,2,2);
plot(out.time_hist, out.vx_hist, 'LineWidth', 2); hold on;
plot(out.time_hist, out.vxref_hist, '--r', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Speed (m/s)');
title('Speed Response');
legend('Actual speed','Target speed','Location','southeast');

subplot(2,2,3);
plot(out.time_hist, out.ey_hist, 'LineWidth', 2); hold on;
yline(lane_bound, '--r');
yline(-lane_bound, '--r');
grid on;
xlabel('Time (s)');
ylabel('Lateral error e_y (m)');
title('Lateral Error');

subplot(2,2,4);
plot(out.time_hist, rad2deg(out.delta_hist), 'LineWidth', 2); hold on;
grid on;
xlabel('Time (s)');
ylabel('Steering angle \delta (deg)');
title('Front Wheel Steering Input');

end
