clc; clear; close all;

%  LQR constant-speed cruise control
%  x = [ev; ey; epsi; vy; r]
%  u = [a; delta]

rootDir = fileparts(mfilename('fullpath'));
addpath(genpath(rootDir));

fprintf('==============================================\n');
fprintf('LQR constant-speed cruise control\n');
fprintf('==============================================\n');
fprintf('Road Tpye:\n');
fprintf('1 - Straight\n');
fprintf('2 - Single-sine curved road.\n');
fprintf('3 - Double-sine curved road.\n');
fprintf('4 - S-shape road\n');
fprintf('----------------------------------------------\n');

%%Parameter
useInteractiveInput = true;
doBatchAnalysis = true;
doWeightSweep   = true;

[P, road, dist] = defaultParams();

if useInteractiveInput
    [P, road, dist] = interactiveInputs(P, road, dist);
end

[P.x0, P.y0, P.psi0] = autoAlignInitialPose(road, P.start_s, P.start_offset);

fprintf('\nAutomatically set the initial pose:\n');
fprintf('x0   = %.3f m\n', P.x0);
fprintf('y0   = %.3f m\n', P.y0);
fprintf('psi0 = %.3f deg\n', rad2deg(P.psi0));

%%LQRcontroller
K = buildLQRController(P);

fprintf('----------------------------------------------\n');
fprintf('LQR gain K = \n');
disp(K);

%%closedloop simulation
out = simulateScenario(P, road, dist, K);

%%animation
animateSimulation(out, P, road);

%%Simulation Results
printMetrics(out.metrics, out.vx_hist, out.vxref_hist);
plotStaticResults(out, P, road);

%%Comfort
plotComfort(out);

%%Robust Analysis
if doBatchAnalysis
    runBatchAnalysis(P, road, dist);
end

%%LQR weight tradeoff
if doWeightSweep
    runWeightSweep(P, road, dist);
end
