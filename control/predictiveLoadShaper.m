function shape = predictiveLoadShaper(gH,FzRide,FyH,lambdaBound,muBound,deltaH,cfg)
%PREDICTIVELOADSHAPER Compute ride and maximally shaped PDAR envelopes.

Np = size(gH,2);
P = eye(4)-ones(4)/4;
shape.gammaRide = zeros(1,Np);
shape.gammaStar = zeros(1,Np);
shape.FzStar = FzRide;
shape.sensitivity = zeros(4,Np);
shape.solverStatus = zeros(1,Np);

previousShift = zeros(4,1);
for j = 1:Np
    [g0,~,stat] = directionalMargin(gH(:,j),FzRide(:,j),FyH(:,j), ...
        lambdaBound,muBound,deltaH(j),cfg,true);
    shape.gammaRide(j) = g0;
    shape.solverStatus(j) = stat;

    q = zeros(4,1);
    epsF = cfg.control.fdLoad;
    for i = 1:4
        di = P(:,i);
        Fp = max(FzRide(:,j)+epsF*di,0.05*cfg.vehicle.Fz0);
        Fm = max(FzRide(:,j)-epsF*di,0.05*cfg.vehicle.Fz0);
        gp = directionalMargin(gH(:,j),Fp,FyH(:,j),lambdaBound,muBound,deltaH(j),cfg,true);
        gm = directionalMargin(gH(:,j),Fm,FyH(:,j),lambdaBound,muBound,deltaH(j),cfg,true);
        q(i) = (gp-gm)/(2*epsF);
    end
    s = P*q;
    shape.sensitivity(:,j) = s;

    reach = cfg.susp.loadShiftMax*(1-exp(-j*cfg.sim.Ts/cfg.susp.tau));
    if max(abs(s)) < 1e-12
        desiredShift = zeros(4,1);
    else
        desiredShift = reach*s/max(abs(s));
    end

    maxStep = cfg.susp.loadShiftRate*cfg.sim.Ts;
    desiredShift = previousShift + min(max(desiredShift-previousShift,-maxStep),maxStep);
    desiredShift = P*desiredShift;

    bestGamma = g0;
    bestShift = zeros(4,1);
    trust = cfg.control.trustInitial;
    for iter = 1:cfg.control.maxShapingIterations
        accepted = false;
        for alpha = [0.25,0.50,0.75,1.00]*trust
            dF = alpha*desiredShift;
            Ftest = FzRide(:,j)+dF;
            if any(Ftest < 0.05*cfg.vehicle.Fz0)
                continue;
            end
            gt = directionalMargin(gH(:,j),Ftest,FyH(:,j), ...
                lambdaBound,muBound,deltaH(j),cfg,true);
            if gt > bestGamma+1e-7
                bestGamma = gt;
                bestShift = dF;
                accepted = true;
            end
        end
        if accepted
            break;
        end
        trust = 0.5*trust;
        if trust < cfg.control.trustMin
            break;
        end
    end
    shape.FzStar(:,j) = FzRide(:,j)+bestShift;
    shape.gammaStar(j) = bestGamma;
    previousShift = bestShift;
end

[shape.GammaRide,shape.criticalIndexRide] = min(shape.gammaRide);
[shape.GammaStar,shape.criticalIndexStar] = min(shape.gammaStar);
shape.criticalIndex = shape.criticalIndexRide;
shape.gain = shape.GammaStar-shape.GammaRide;
end

