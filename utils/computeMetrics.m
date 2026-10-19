function M = computeMetrics(H,scn,cfg)
%COMPUTEMETRICS Motion, reserve, energy, feasibility, and timing metrics.

yError = H.Y-H.yRef;
rError = H.r-H.rRef;
M.lateralRMSE = sqrt(mean(yError.^2,'omitnan'));
M.yawRMSE = sqrt(mean(rError.^2,'omitnan'));
M.peakSideslipDeg = rad2deg(max(abs(H.beta),[],'omitnan'));
M.peakRollDeg = rad2deg(max(abs(H.phi),[],'omitnan'));
M.peakRho = max(H.rhoMax,[],'omitnan');
M.minRealizedPDAR = min(H.gammaActual,[],'omitnan');
M.minPredictedPDAR = min(H.GammaRide,[],'omitnan');
M.maxASRGain = max(H.ASRGain,[],'omitnan');
M.meanASRGain = mean(H.ASRGain,'omitnan');
M.suspensionEnergyKJ = H.energy(end)/1000;
M.uncertifiedFraction = mean(H.modeCode==3);
M.fallbackCount = sum(H.solverStatus<=0);
M.constraintViolationCount = sum(H.constraintViolation>0);

idxActivate = find(H.modeCode==2,1,'first');
idxLoss = find(H.GammaRide<1,1,'first');
if isempty(idxActivate), M.activationTime = NaN; else, M.activationTime = H.t(idxActivate); end
if isempty(idxLoss), M.firstLossTime = NaN; else, M.firstLossTime = H.t(idxLoss); end
if isempty(idxActivate) || isempty(idxLoss)
    M.warningLeadTime = NaN;
else
    M.warningLeadTime = H.t(idxLoss)-H.t(idxActivate);
end

post = H.t>=scn.faultTime;
within = abs(yError)<=0.10 & abs(rError)<=deg2rad(1.0);
M.recoveryTime = NaN;
idxPost = find(post);
for k = idxPost
    j2 = min(numel(H.t),k+round(0.5/cfg.sim.Ts));
    if all(within(k:j2))
        M.recoveryTime = H.t(k)-scn.faultTime;
        break;
    end
end

M.falseSafeRate = mean((H.GammaRide>=1) & (H.gammaActual<1));
M.falseRecoverableRate = mean((H.modeCode==2) & ...
    (H.gammaActual<1+cfg.control.deltaGamma));
M.recoverable = max(abs(yError),[],'omitnan')<=0.50 && ...
    max(abs(H.beta),[],'omitnan')<=deg2rad(6) && M.peakRho<=1.001 && ...
    M.constraintViolationCount==0 && M.fallbackCount < 0.05*numel(H.t);
M.failure = ~M.recoverable;
end

