function printMetrics(metrics, vx_hist, vxref_hist)

fprintf('\n================ Simulation Results ================\n');
fprintf('Final speed                         = %.3f m/s\n', vx_hist(end));
fprintf('Final speed error                   = %.3f m/s\n', vx_hist(end)-vxref_hist(end));
fprintf('Speed overshoot (relative to final reference) = %.2f %%\n', metrics.speedOvershootPct);
fprintf('Speed settling time                 = %.3f s\n', metrics.speedSettlingTime);
fprintf('Steady-state speed error            = %.3f m/s\n', metrics.speedSteadyError);

fprintf('Maximum lateral error               = %.3f m\n', metrics.maxEy);
fprintf('Lateral error RMS                   = %.3f m\n', metrics.rmsEy);
fprintf('Steady-state lateral error          = %.3f m\n', metrics.ssEy);

fprintf('Maximum front wheel steering angle  = %.3f deg\n', metrics.maxDeltaDeg);
fprintf('Maximum longitudinal acceleration   = %.3f m/s^2\n', metrics.maxAx);
fprintf('Maximum lateral acceleration        = %.3f m/s^2\n', metrics.maxAy);
fprintf('Maximum jerk                        = %.3f m/s^3\n', metrics.maxJerk);
fprintf('RMS jerk                            = %.3f m/s^3\n', metrics.rmsJerk);
fprintf('Always stays within the lane        = %d\n', metrics.insideLane);

if metrics.insideLane
    disp('Conclusion: The vehicle always stays within the road boundaries.');
else
    disp('Conclusion: The vehicle goes out of the lane. Please reduce the speed or retune Q and R.');
end
end