function Tresult = runBatchAnalysis(P, road, dist)
%RUNBATCHANALYSIS 多场景鲁棒性分析。

disp(' ');
disp('============= 开始多场景分析 =============');

scenarioList = {};

S.name = 'Baseline';
S.dist = dist;
S.road = road;
scenarioList{end+1} = S;

S.name = 'Uphill +4deg';
S.dist = dist;
S.dist.gradeType = 1;
S.dist.gradeValue = deg2rad(4);
S.road = road;
scenarioList{end+1} = S;

S.name = 'Downhill -4deg';
S.dist = dist;
S.dist.gradeType = 1;
S.dist.gradeValue = deg2rad(-4);
S.road = road;
scenarioList{end+1} = S;

S.name = 'Mass +20%';
S.dist = dist;
S.dist.massScale = 1.2;
S.road = road;
scenarioList{end+1} = S;

S.name = 'Mass -20%';
S.dist = dist;
S.dist.massScale = 0.8;
S.road = road;
scenarioList{end+1} = S;

S.name = 'CfCr -30%';
S.dist = dist;
S.dist.CfScale = 0.7;
S.dist.CrScale = 0.7;
S.road = road;
scenarioList{end+1} = S;

S.name = 'Curvature Boost';
S.dist = dist;
S.dist.useCurvatureBoost = true;
S.dist.curvBoostStart = 45;
S.dist.curvBoostEnd   = 100;
S.dist.curvBoostScale = 2.0;
S.road = road;
scenarioList{end+1} = S;

S.name = 'Speed Step 20to25';
S.dist = dist;
S.dist.useSpeedStep = true;
S.dist.stepTime = 4;
S.dist.vxRef2 = 25;
S.road = road;
scenarioList{end+1} = S;

numScenarios = length(scenarioList);
resultNames = strings(numScenarios,1);
maxEyVec    = zeros(numScenarios,1);
rmsEyVec    = zeros(numScenarios,1);
ssEvVec     = zeros(numScenarios,1);
settleVec   = zeros(numScenarios,1);
maxDeltaVec = zeros(numScenarios,1);
jerkVec     = zeros(numScenarios,1);
laneVec     = zeros(numScenarios,1);

for i = 1:numScenarios
    out = simulateScenario(P, scenarioList{i}.road, scenarioList{i}.dist);

    resultNames(i) = string(scenarioList{i}.name);
    maxEyVec(i)    = out.metrics.maxEy;
    rmsEyVec(i)    = out.metrics.rmsEy;
    ssEvVec(i)     = out.metrics.speedSteadyError;
    settleVec(i)   = out.metrics.speedSettlingTime;
    maxDeltaVec(i) = out.metrics.maxDeltaDeg;
    jerkVec(i)     = out.metrics.rmsJerk;
    laneVec(i)     = out.metrics.insideLane;
end

Tresult = table(resultNames, maxEyVec, rmsEyVec, ssEvVec, settleVec, ...
                maxDeltaVec, jerkVec, laneVec, ...
    'VariableNames', {'Scenario','MaxEy_m','RMSEy_m','SpeedSSerr_mps', ...
                      'SpeedSettle_s','MaxDelta_deg','RMSJerk','InsideLane'});
disp(Tresult);

figure('Color','w','Position',[220 120 1100 650]);
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

x = 1:length(resultNames);

nexttile;
plot(x, maxEyVec, '-o', 'LineWidth', 1.8, 'MarkerSize', 7); grid on;
xticks(x); xticklabels(resultNames); xtickangle(30);
ylabel('Max |e_y| (m)'); title('maximum lateral error comparison');

nexttile;
plot(x, settleVec, '-o', 'LineWidth', 1.8, 'MarkerSize', 7); grid on;
xticks(x); xticklabels(resultNames); xtickangle(30);
ylabel('Settling Time (s)'); title('Speed settling time comparison.');

nexttile;
plot(x, maxDeltaVec, '-o', 'LineWidth', 1.8, 'MarkerSize', 7); grid on;
xticks(x); xticklabels(resultNames); xtickangle(30);
ylabel('Max \delta (deg)'); title('Maximum steering angle comparison');

nexttile;
plot(x, jerkVec, '-o', 'LineWidth', 1.8, 'MarkerSize', 7); grid on;
xticks(x); xticklabels(resultNames); xtickangle(30);
ylabel('RMS total jerk'); title('Comfort comparison');
end
