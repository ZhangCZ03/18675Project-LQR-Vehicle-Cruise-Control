function [x_road, y_road, y_up, y_dn] = generateRoadForPlot(x_hist, road, lane_bound)

x_plot_max = max(x_hist) + 20;
if ~isfinite(x_plot_max) || x_plot_max <= 0
    x_plot_max = 100;
end

x_road = linspace(0, x_plot_max, 1500);
y_road = zeros(size(x_road));

for i = 1:length(x_road)
    [y_road(i), ~, ~] = roadProfile(x_road(i), road);
end

y_up = y_road + lane_bound;
y_dn = y_road - lane_bound;
end
