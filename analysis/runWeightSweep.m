function Tsweep = runWeightSweep(P, road, dist)

disp(' ');
disp('LQR Weight scannning');

qy_list   = [100, 500, 1200]; 
rdel_list = [5, 15, 45];  

numCases = length(qy_list)*length(rdel_list);
cnt = 1;
sweepName = strings(numCases,1);
sweepMaxEy = zeros(numCases,1);
sweepJerk  = zeros(numCases,1);
sweepDelta = zeros(numCases,1);

for iq = 1:length(qy_list)
    for ir = 1:length(rdel_list)
        Ptmp = P;
        Ptmp.qy   = qy_list(iq);
        Ptmp.rdel = rdel_list(ir);

        out = simulateScenario(Ptmp, road, dist);

        sweepName(cnt)  = "qy=" + qy_list(iq) + ",rdel=" + rdel_list(ir);
        sweepMaxEy(cnt) = out.metrics.maxEy;
        sweepJerk(cnt)  = out.metrics.rmsJerk;
        sweepDelta(cnt) = out.metrics.maxDeltaDeg;
        cnt = cnt + 1;
    end
end

Tsweep = table(sweepName, sweepMaxEy, sweepJerk, sweepDelta, ...
    'VariableNames', {'Weights','MaxEy_m','RMSJerk','MaxDelta_deg'});
disp(Tsweep);

figure('Color','w','Position',[240 150 1000 520]);
tiledlayout(1,3,'Padding','compact','TileSpacing','compact');

x = 1:length(sweepName);

nexttile;
plot(x, sweepMaxEy, '-o', 'LineWidth', 1.8, 'MarkerSize', 7); grid on;
xticks(x);
xticklabels(sweepName);
xtickangle(45);
ylabel('Max |e_y| (m)');
title('Tracking Performance');

nexttile;
plot(x, sweepJerk, '-o', 'LineWidth', 1.8, 'MarkerSize', 7); grid on;
xticks(x);
xticklabels(sweepName);
xtickangle(45);
ylabel('RMS jerk');
title('Comfort');

nexttile;
plot(x, sweepDelta, '-o', 'LineWidth', 1.8, 'MarkerSize', 7); grid on;
xticks(x);
xticklabels(sweepName);
xtickangle(45);
ylabel('Max \delta (deg)');
title('Control Effort');

disp('===== Weight Sweep Case Mapping =====');
for i = 1:length(sweepName)
    fprintf('%2d : %s\n', i, sweepName(i));
end
end
