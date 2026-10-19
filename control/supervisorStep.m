function [sup,state] = supervisorStep(method,demand,x,ref,lambdaEst,muEst,state,cfg)
%SUPERVISORSTEP Reachable-tube prediction, recoverability gate, and target load.

method = upper(string(method));
Np = cfg.control.Np;
if any(method == ["R_ASR","A1","J_MPC","HEALTHY","NO_FTC", ...
        "TORQUE_FTC","RT_FTC","PARALLEL_ICC","A0"])
    Np = 1;
elseif method == "A2"
    Np = cfg.control.Np;
end

gH = repmat(demand.g,1,Np);
deltaH = repmat(ref.delta,1,Np);
FzRide = repmat(x.Fz,1,Np);
FyH = repmat(x.Fy,1,Np);

% Mild preview variation avoids a frozen demand sequence in A2-A4.
if any(method == ["P_ASR","A2","A3","A4"])
    previewGain = linspace(1,1.10,Np);
    gH(2,:) = gH(2,:).*previewGain;
end

lambdaBound = max(lambdaEst-cfg.control.lambdaUncertainty,0);
muBound = max(muEst-cfg.control.muUncertainty,0.05);
shape = predictiveLoadShaper(gH,FzRide,FyH,lambdaBound,muBound,deltaH,cfg);

thetaMinus = 1+cfg.control.deltaGamma-cfg.control.epsilonH;
thetaPlus = 1+cfg.control.deltaGamma+cfg.control.epsilonH;

if shape.GammaRide >= thetaPlus
    candidate = "certified";
elseif shape.GammaRide <= thetaMinus && shape.GammaStar >= thetaPlus
    candidate = "shape-recoverable";
elseif shape.GammaStar <= thetaMinus
    candidate = "uncertified";
else
    candidate = state.mode;
end

if candidate == state.candidate
    state.dwellCount = state.dwellCount+1;
else
    state.candidate = candidate;
    state.dwellCount = 1;
end
if state.dwellCount >= cfg.control.dwellSamples
    state.mode = candidate;
end

switch method
    case {"P_ASR","A4"}
        modeOut = state.mode;
    case "A3"
        modeOut = "shape-recoverable";
    case {"R_ASR","A1"}
        if shape.GammaRide < 1+cfg.control.deltaGamma
            modeOut = "shape-recoverable";
        else
            modeOut = "certified";
        end
    case "A2"
        modeOut = "shape-recoverable";
        shape.FzStar = FzRide; % Preview without dynamically reachable load shaping.
        shape.GammaStar = shape.GammaRide;
        shape.gain = 0;
    case "J_MPC"
        modeOut = "joint-MPC";
    otherwise
        modeOut = "inactive";
end

FzTarget = FzRide(:,1);
if any(method == ["P_ASR","A3","A4","R_ASR","A1"])
    if modeOut == "shape-recoverable" || ...
            (modeOut == "uncertified" && shape.gain>cfg.control.epsilonG)
        FzTarget = shape.FzStar(:,1);
    end
elseif method == "J_MPC"
    FzTarget = 0.55*shape.FzStar(:,1)+0.45*FzRide(:,1);
end

sup.mode = modeOut;
sup.candidate = candidate;
sup.GammaRide = shape.GammaRide;
sup.GammaStar = shape.GammaStar;
sup.gain = shape.gain;
sup.gammaRide = shape.gammaRide;
sup.gammaStar = shape.gammaStar;
sup.criticalIndex = shape.criticalIndex;
sup.FzTarget = FzTarget;
sup.sensitivity = shape.sensitivity(:,shape.criticalIndex);
sup.degradedFlag = modeOut == "uncertified";
sup.solverStatus = min(shape.solverStatus);
end
