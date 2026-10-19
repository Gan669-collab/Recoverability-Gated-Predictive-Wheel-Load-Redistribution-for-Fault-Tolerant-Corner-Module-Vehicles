function [lambdaTrue, lambdaEstimated] = faultSignal(t, scn)
%FAULTSIGNAL True and delayed/biased motor health factors.

lambdaTrue = ones(4,1);
if t >= scn.faultTime
    lambdaTrue(scn.faultWheel) = scn.lambdaFault;
end

tObserved = t - scn.healthDelay;
lambdaEstimated = ones(4,1);
if tObserved >= scn.faultTime
    lambdaEstimated(scn.faultWheel) = scn.lambdaFault;
end
lambdaEstimated = min(max(lambdaEstimated + scn.healthBias,0),1);
end

