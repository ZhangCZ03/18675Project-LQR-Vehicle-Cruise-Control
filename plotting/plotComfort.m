function plotComfort(out)


figure('Color','w','Position',[180 120 1100 700]);

subplot(3,1,1);
plot(out.time_hist, out.ax_hist, 'LineWidth', 1.8); grid on;
xlabel('Time (s)'); ylabel('a_x (m/s^2)');
title('Longitudinal acceleration');

subplot(3,1,2);
plot(out.time_hist, out.ay_hist, 'LineWidth', 1.8); grid on;
xlabel('Time (s)'); ylabel('a_y (m/s^2)');
title('Lateral acceleration');

subplot(3,1,3);
plot(out.time_hist, out.jerk_hist, 'LineWidth', 1.8); grid on;
xlabel('Time (s)'); ylabel('jerk (m/s^3)');
title('Total jerk');
end
