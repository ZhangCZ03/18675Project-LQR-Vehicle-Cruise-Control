function metrics = computeMetrics(t, vx, vxref, ey, delta, ax, ay, jerk, lane_bound)

speed_err = vx - vxref;
final_ref = vxref(end);

if abs(final_ref) < 1e-6
    speedOvershootPct = 0;
else
    speedOvershootPct = max(0, (max(vx)-final_ref)/abs(final_ref)*100);
end

band = 0.02 * max(abs(final_ref), 1);   % 2% band
settlingTime = NaN;
for i = 1:length(t)
    if all(abs(speed_err(i:end)) <= band)
        settlingTime = t(i);
        break;
    end
end

nTail = max(10, round(0.1*length(t)));
nTail = min(nTail, length(t));

metrics.speedOvershootPct = speedOvershootPct;
metrics.speedSettlingTime = settlingTime;
metrics.speedSteadyError  = mean(speed_err(end-nTail+1:end));

nSkip = max(1, round(0.15*length(ey)));   % skip first 15%
ey_eval = ey(nSkip:end);
if isempty(ey_eval)
    ey_eval = ey;
end

metrics.maxEy      = max(abs(ey));
metrics.maxEyAfter = max(abs(ey_eval));
metrics.rmsEy      = sqrt(mean(ey_eval.^2));
metrics.ssEy       = mean(ey(end-nTail+1:end));

idx_enter = find(abs(speed_err) <= band, 1, 'first');
if isempty(idx_enter)
    idx_enter = 1;
end

delta_eval = delta(idx_enter:end);
ax_eval    = ax(idx_enter:end);
ay_eval    = ay(idx_enter:end);
jerk_eval  = jerk(idx_enter:end);

metrics.maxDeltaDeg = max(abs(rad2deg(delta_eval)));
metrics.maxAx       = max(abs(ax_eval));
metrics.maxAy       = max(abs(ay_eval));
metrics.maxJerk     = max(abs(jerk_eval));
metrics.rmsJerk     = sqrt(mean(jerk_eval.^2));
metrics.insideLane  = all(abs(ey) <= lane_bound);
end
