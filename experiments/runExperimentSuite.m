function R = runExperimentSuite(cfg)
%RUNEXPERIMENTSUITE Execute E1-E10 with common seeds and matched constraints.

isPaper = lower(cfg.experiment.mode) == "paper";
if isPaper
    lambdaGrid = cfg.experiment.lambdaGridPaper;
    muGrid = cfg.experiment.muGridPaper;
    nMC = cfg.experiment.mcCountPaper;
    delayGrid = cfg.experiment.delayGridPaper;
    biasGrid = cfg.experiment.biasGridPaper;
    nDelaySeeds = cfg.experiment.delaySeedsPaper;
    forceGrid = cfg.experiment.forceScalePaper;
    tauGrid = cfg.experiment.tauScalePaper;
    nMismatchSeeds = cfg.experiment.mismatchSeedsPaper;
    horizonGrid = [5,10,15,20];
    reserveGrid = [0,0.05,0.10,0.15];
else
    lambdaGrid = 0:0.25:1;
    muGrid = [0.30,0.45,0.60,0.75];
    nMC = 8;
    delayGrid = [0,0.10,0.20];
    biasGrid = [-0.15,0,0.15];
    nDelaySeeds = 1;
    forceGrid = [0.60,1.00,1.20];
    tauGrid = [0.75,1.00,2.00];
    nMismatchSeeds = 1;
    horizonGrid = [5,10,15];
    reserveGrid = [0,0.10,0.15];
    cfg.control.Np = 10;
    cfg.control.supervisorStride = 5;
end

baseDLC = makeScenario("DLC",cfg);
baseSWD = makeScenario("SWD",cfg);
baseSplit = makeScenario("SPLIT",cfg);

fprintf('E1-E4: deterministic simulations...\n');
for method = cfg.experiment.methods
    key = char(method);
    R.deterministic.DLC.(key) = simulateCase(baseDLC,method,cfg,cfg.sim.seed);
    R.deterministic.SWD.(key) = simulateCase(baseSWD,method,cfg,cfg.sim.seed+1);
    R.deterministic.SPLIT.(key) = simulateCase(baseSplit,method,cfg,cfg.sim.seed+2);
end
R.audit = buildAudit(R.deterministic.DLC.P_ASR,cfg);

fprintf('E5: fault-severity/friction boundary...\n');
R.boundary.lambda = lambdaGrid;
R.boundary.mu = muGrid;
R.boundary.methods = cfg.experiment.boundaryMethods;
for im = 1:numel(cfg.experiment.boundaryMethods)
    method = cfg.experiment.boundaryMethods(im);
    recoverable = false(numel(muGrid),numel(lambdaGrid));
    minPDAR = nan(size(recoverable));
    for ia = 1:numel(lambdaGrid)
        for iu = 1:numel(muGrid)
            scn = baseDLC;
            scn.lambdaFault = lambdaGrid(ia);
            scn.muLeft = muGrid(iu);
            scn.muRight = muGrid(iu);
            out = simulateCase(scn,method,cfg,100000+1000*im+100*iu+ia);
            recoverable(iu,ia) = out.metrics.recoverable;
            minPDAR(iu,ia) = out.metrics.minRealizedPDAR;
        end
    end
    key = char(method);
    R.boundary.recoverable.(key) = recoverable;
    R.boundary.minPDAR.(key) = minPDAR;
    R.boundary.areaRatio.(key) = mean(recoverable,'all');
end

fprintf('E6: paired Monte Carlo (%d seeds)...\n',nMC);
R.monteCarlo.methods = cfg.experiment.mcMethods;
R.monteCarlo.seeds = cfg.sim.seed+(1:nMC);
metricNames = metricFieldNames();
for name = metricNames
    R.monteCarlo.metrics.(name) = nan(nMC,numel(cfg.experiment.mcMethods));
end
for iseed = 1:nMC
    scn = sampleUncertainScenario(baseDLC,R.monteCarlo.seeds(iseed),cfg);
    for im = 1:numel(cfg.experiment.mcMethods)
        out = simulateCase(scn,cfg.experiment.mcMethods(im),cfg,R.monteCarlo.seeds(iseed));
        R.monteCarlo = assignMetrics(R.monteCarlo,out.metrics,iseed,im,metricNames);
    end
end

fprintf('E7: A0-A4 mechanism ablation...\n');
R.ablation.methods = ["A0","A1","A2","A3","A4"];
for im = 1:numel(R.ablation.methods)
    out = simulateCase(baseDLC,R.ablation.methods(im),cfg,cfg.sim.seed+700+im);
    R.ablation.outputs.(char(R.ablation.methods(im))) = out;
    R.ablation.metricMatrix(im,:) = metricRow(out.metrics);
end
R.ablation.metricNames = ["lateralRMSE","yawRMSE","peakRho", ...
    "minRealizedPDAR","suspensionEnergyKJ","failure"];

fprintf('E8: horizon/reserve sensitivity...\n');
R.sensitivity.horizons = horizonGrid;
R.sensitivity.reserve = reserveGrid;
R.sensitivity.minPDAR = nan(numel(horizonGrid),numel(reserveGrid));
R.sensitivity.energy = nan(size(R.sensitivity.minPDAR));
R.sensitivity.fallbacks = nan(size(R.sensitivity.minPDAR));
for ih = 1:numel(horizonGrid)
    for ir = 1:numel(reserveGrid)
        cfg8 = cfg;
        cfg8.control.Np = horizonGrid(ih);
        cfg8.control.deltaGamma = reserveGrid(ir);
        out = simulateCase(baseDLC,"P_ASR",cfg8,cfg.sim.seed+800+10*ih+ir);
        R.sensitivity.minPDAR(ih,ir) = out.metrics.minRealizedPDAR;
        R.sensitivity.energy(ih,ir) = out.metrics.suspensionEnergyKJ;
        R.sensitivity.fallbacks(ih,ir) = out.metrics.fallbackCount;
    end
end

fprintf('E9: delayed/biased health input...\n');
delayMethods = ["RT_FTC","J_MPC","R_ASR","P_ASR"];
R.delay.delay = delayGrid;
R.delay.bias = biasGrid;
R.delay.methods = delayMethods;
sz9 = [numel(delayGrid),numel(biasGrid),numel(delayMethods)];
R.delay.failureProbability = zeros(sz9);
R.delay.minPDAR = zeros(sz9);
R.delay.peakRho = zeros(sz9);
for id = 1:numel(delayGrid)
    for ib = 1:numel(biasGrid)
        for im = 1:numel(delayMethods)
            fail = zeros(nDelaySeeds,1); pdar = fail; rho = fail;
            for iseed = 1:nDelaySeeds
                % The scenario seed is independent of the method index so that
                % every controller is evaluated on the same realization.
                scenarioSeed = 900000+10000*id+1000*ib+iseed;
                scn = sampleUncertainScenario(baseDLC,scenarioSeed,cfg);
                scn.healthDelay = delayGrid(id);
                scn.healthBias = biasGrid(ib);
                out = simulateCase(scn,delayMethods(im),cfg,900000+iseed);
                fail(iseed)=out.metrics.failure;
                pdar(iseed)=out.metrics.minRealizedPDAR;
                rho(iseed)=out.metrics.peakRho;
            end
            R.delay.failureProbability(id,ib,im)=mean(fail);
            R.delay.minPDAR(id,ib,im)=mean(pdar);
            R.delay.peakRho(id,ib,im)=mean(rho);
        end
    end
end

fprintf('E10: suspension force/lag mismatch...\n');
mismatchMethods = ["Parallel_ICC","J_MPC","R_ASR","P_ASR"];
R.mismatch.forceScale = forceGrid;
R.mismatch.tauScale = tauGrid;
R.mismatch.methods = mismatchMethods;
sz10 = [numel(forceGrid),numel(tauGrid),numel(mismatchMethods)];
R.mismatch.failureProbability = zeros(sz10);
R.mismatch.falseRecoverableRate = zeros(sz10);
R.mismatch.minPDAR = zeros(sz10);
R.mismatch.energy = zeros(sz10);
for iF = 1:numel(forceGrid)
    for it = 1:numel(tauGrid)
        for im = 1:numel(mismatchMethods)
            fail=zeros(nMismatchSeeds,1); fr=fail; pdar=fail; energy=fail;
            for iseed = 1:nMismatchSeeds
                scenarioSeed = 1000000+10000*iF+1000*it+iseed;
                scn = sampleUncertainScenario(baseDLC,scenarioSeed,cfg);
                scn.forceScale = forceGrid(iF);
                scn.tauScale = tauGrid(it);
                out = simulateCase(scn,mismatchMethods(im),cfg,1000000+iseed);
                fail(iseed)=out.metrics.failure;
                fr(iseed)=out.metrics.falseRecoverableRate;
                pdar(iseed)=out.metrics.minRealizedPDAR;
                energy(iseed)=out.metrics.suspensionEnergyKJ;
            end
            R.mismatch.failureProbability(iF,it,im)=mean(fail);
            R.mismatch.falseRecoverableRate(iF,it,im)=mean(fr);
            R.mismatch.minPDAR(iF,it,im)=mean(pdar);
            R.mismatch.energy(iF,it,im)=mean(energy);
        end
    end
end

R.experimentSettings.mode = cfg.experiment.mode;
R.experimentSettings.nMC = nMC;
R.experimentSettings.nDelaySeeds = nDelaySeeds;
R.experimentSettings.nMismatchSeeds = nMismatchSeeds;
R.config = cfg;
end

function audit = buildAudit(out,cfg)
H = out.history;
sample = 1:max(1,round(0.10/cfg.sim.Ts)):numel(H.t);
near = find(H.GammaRide>=0.8 & H.GammaRide<=1.2);
sample = unique([sample,near]);
audit.predicted = H.GammaRide(sample);
audit.realized = H.gammaActual(sample);
audit.gap = (audit.realized-audit.predicted)./max(audit.realized,1e-6);
audit.falseSafe = (audit.predicted>=1)&(audit.realized<1);
audit.falseRecoverable = H.modeCode(sample)==2 & ...
    audit.realized<1+cfg.control.deltaGamma;
audit.time = H.t(sample);
end

function names = metricFieldNames()
names = ["lateralRMSE","yawRMSE","peakSideslipDeg","peakRollDeg", ...
    "peakRho","minRealizedPDAR","maxASRGain","suspensionEnergyKJ", ...
    "warningLeadTime","failure","fallbackCount"];
end

function mc = assignMetrics(mc,M,i,j,names)
for name = names
    key = char(name);
    values = mc.metrics.(key);
    values(i,j) = M.(key);
    mc.metrics.(key) = values;
end
end

function row = metricRow(M)
row = [M.lateralRMSE,M.yawRMSE,M.peakRho,M.minRealizedPDAR, ...
    M.suspensionEnergyKJ,M.failure];
end
