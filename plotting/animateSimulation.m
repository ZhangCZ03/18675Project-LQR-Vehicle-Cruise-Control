function animateSimulation(out, P, road)

lane_bound = P.lane_width/2;
[x_road, y_road, y_up, y_dn] = generateRoadForPlot(out.x_hist, road, lane_bound);

car_L = 4.5;
car_W = 1.8;

fig = figure('Color','w','Position',[80 60 1200 760]);
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

ax1 = nexttile(1,[2 1]);
hold(ax1,'on'); grid(ax1,'on'); axis(ax1,'equal');
xlabel(ax1,'x (m)');
ylabel(ax1,'y (m)');
title(ax1,'Tracking animation');

plot(ax1, x_road, y_up, '--', 'LineWidth', 1.2);
plot(ax1, x_road, y_dn, '--', 'LineWidth', 1.2);
plot(ax1, x_road, y_road, '-.', 'LineWidth', 1.0);

trajLine = plot(ax1, out.x_hist(1), out.y_hist(1), 'LineWidth', 2);
[Xcar, Ycar] = getCarShape(out.x_hist(1), out.y_hist(1), out.psi_hist(1), car_L, car_W);
carPatch = patch(ax1, Xcar, Ycar, [0.2 0.6 0.9], 'FaceAlpha', 0.85);
headLine = plot(ax1, ...
    [out.x_hist(1), out.x_hist(1)+car_L/2*cos(out.psi_hist(1))], ...
    [out.y_hist(1), out.y_hist(1)+car_L/2*sin(out.psi_hist(1))], 'r', 'LineWidth', 2);

infoText = text(ax1, out.x_hist(1)+2, out.y_hist(1)+2, '', 'FontSize', 11, 'FontWeight', 'bold');

xlim(ax1, [0, 60]);
ylim(ax1, [min(y_dn)-2, max(y_up)+2]);

ax2 = nexttile(2);
hold(ax2,'on'); grid(ax2,'on');
xlabel(ax2,'Time (s)');
ylabel(ax2,'Speed (m/s)');
title(ax2,'Constant-speed cruise control process');
plot(ax2, out.time_hist, out.vxref_hist, '--r', 'LineWidth', 1.5);
speedLine = plot(ax2, out.time_hist(1), out.vx_hist(1), 'LineWidth', 2);
legend(ax2,'Target speed','Actual speed','Location','southeast');
xlim(ax2,[0 P.T]);
ylim(ax2,[0, max(max(out.vx_hist)+2, max(out.vxref_hist)+2)]);

%lateral error
ax3 = nexttile(4);
hold(ax3,'on'); grid(ax3,'on');
xlabel(ax3,'Time (s)');
ylabel(ax3,'e_y (m)');
title(ax3,'lateral error');
eyLine = plot(ax3, out.time_hist(1), out.ey_hist(1), 'LineWidth', 2);
yline(ax3, lane_bound, '--r', 'LineWidth', 1.0);
yline(ax3, -lane_bound, '--r', 'LineWidth', 1.0);
xlim(ax3,[0 P.T]);

%Steering angle
fig2 = figure('Color','w','Position',[180 120 900 380]);
hold on; grid on;
xlabel('Time (s)');
ylabel('Front wheel steering angle \delta (deg)');
title('Steering control input');
deltaLine = plot(out.time_hist(1), rad2deg(out.delta_hist(1)), 'LineWidth', 2);
xlim([0 P.T]);

drawStep = 5;

for k = 1:drawStep:length(out.time_hist)
    x_now = out.x_hist(k);
    idxWin = abs(x_road - x_now) < 35;
    if any(idxWin)
        y_min = min(y_dn(idxWin)) - 2;
        y_max = max(y_up(idxWin)) + 2;
    else
        y_min = min(y_dn) - 2;
        y_max = max(y_up) + 2;
    end

    if x_now > 30
        xlim(ax1, [x_now-30, x_now+30]);
    end
    ylim(ax1, [y_min, y_max]);

    set(trajLine, 'XData', out.x_hist(1:k), 'YData', out.y_hist(1:k));

    [Xcar, Ycar] = getCarShape(out.x_hist(k), out.y_hist(k), out.psi_hist(k), car_L, car_W);
    set(carPatch, 'XData', Xcar, 'YData', Ycar);

    set(headLine, ...
        'XData', [out.x_hist(k), out.x_hist(k)+car_L/2*cos(out.psi_hist(k))], ...
        'YData', [out.y_hist(k), out.y_hist(k)+car_L/2*sin(out.psi_hist(k))]);

    str = sprintf(['t = %.2f s\n' ...
                   'v = %.2f m/s\n' ...
                   'v_{ref} = %.2f m/s\n' ...
                   'e_y = %.2f m\n' ...
                   'e_{\\psi} = %.2f deg\n' ...
                   '\\delta = %.2f deg'], ...
                   out.time_hist(k), out.vx_hist(k), out.vxref_hist(k), out.ey_hist(k), ...
                   rad2deg(out.epsi_hist(k)), rad2deg(out.delta_hist(k)));
    set(infoText, 'Position', [out.x_hist(k)+2, out.y_hist(k)+2, 0], 'String', str);

    set(speedLine, 'XData', out.time_hist(1:k), 'YData', out.vx_hist(1:k));
    set(eyLine, 'XData', out.time_hist(1:k), 'YData', out.ey_hist(1:k));

    figure(fig2);
    set(deltaLine, 'XData', out.time_hist(1:k), 'YData', rad2deg(out.delta_hist(1:k)));

    drawnow;
    figure(fig);
end
end
