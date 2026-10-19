function out = simulateCase(scn, method, cfg, seed)
%SIMULATECASE Run one method and one common-seed manoeuvre.

if nargin < 4
    seed = cfg.sim.seed;
end
rng(seed,'twister');
method = upper(string(method));
x = initialState(scn,cfg);

supState.mode = "certified";
supState.candidate = "certified";
supState.dwellCount = 0;
allocState.lastU = zeros(8,1);
allocState.lastFa = zeros(4,1);
allocState.lastT = zeros(4,1);

t = 0:cfg.sim.Ts:scn.duration;
n = numel(t);
NpMax = cfg.control.Np;
H = initializeHistory(n,NpMax);
lastSup = [];
energy = 0;

for k = 1:n
    tk = t(k);
    ref = referenceInput(tk,x,scn,cfg);
    [lambdaTrue,lambdaEst] = faultSignal(tk,scn);
    if method == "HEALTHY"
        lambdaTrue = ones(4,1);
        lambdaEst = ones(4,1);
    end
    muTrue = [scn.muLeft;scn.muRight;scn.muLeft;scn.muRight];
    muEst = min(max(muTrue*(1+scn.frictionError),0.05),1.2);
    demand = upperDemand(x,ref,cfg);

    updateSupervisor = isempty(lastSup) || mod(k-1,cfg.control.supervisorStride) == 0;
    if updateSupervisor
        [sup,supState] = supervisorStep(method,demand,x,ref,lambdaEst,muEst, ...
            supState,cfg);
        lastSup = sup;
    else
        sup = lastSup;
    end

    [cmd,allocState] = controlAllocator(method,demand,sup,x,ref,lambdaEst, ...
        muEst,allocState,cfg);
    [xNext,aux] = plantStep(x,cmd,ref,lambdaTrue,muTrue,scn,cfg);
    gammaActual = directionalMargin(demand.g,xNext.Fz,xNext.Fy,lambdaTrue, ...
        muTrue,ref.delta,cfg,true);
    energy = energy+cfg.sim.Ts*aux.Pa;

    H = logStep(H,k,tk,xNext,aux,ref,demand,sup,cmd,lambdaTrue,lambdaEst, ...
        muTrue,gammaActual,energy,NpMax);
    x = xNext;
end

out.method = method;
out.scenario = scn;
out.seed = seed;
out.history = H;
out.metrics = computeMetrics(H,scn,cfg);
end

function H = initializeHistory(n,Np)
scalarFields = {'t','X','Y','vx','vy','r','psi','phi','phidot','theta', ...
    'beta','delta','yRef','rRef','rhoMax','GammaRide','GammaStar','ASRGain', ...
    'criticalIndex','gammaActual','demandScale','energy','modeCode','degradedFlag', ...
    'solverStatus','constraintViolation'};
for i = 1:numel(scalarFields)
    H.(scalarFields{i}) = nan(1,n);
end
vectorFields = {'Fz','FzTarget','Fx','Fy','Tcmd','Fa','rho','lambdaTrue', ...
    'lambdaEst','mu','gDemand','sensitivity'};
for i = 1:numel(vectorFields)
    rows = 4;
    if strcmp(vectorFields{i},'gDemand')
        rows = 3;
    end
    H.(vectorFields{i}) = nan(rows,n);
end
H.gammaRideH = nan(Np,n);
H.gammaStarH = nan(Np,n);
end

function H = logStep(H,k,t,x,aux,ref,demand,sup,cmd,lambdaTrue,lambdaEst,mu, ...
    gammaActual,energy,NpMax)
H.t(k)=t; H.X(k)=x.X; H.Y(k)=x.Y; H.vx(k)=x.vx; H.vy(k)=x.vy;
H.r(k)=x.r; H.psi(k)=x.psi; H.phi(k)=x.phi; H.phidot(k)=x.phidot;
H.theta(k)=x.theta; H.beta(k)=aux.beta; H.delta(k)=ref.delta;
H.yRef(k)=ref.y; H.rRef(k)=ref.r; H.rhoMax(k)=aux.rhoMax;
H.GammaRide(k)=sup.GammaRide; H.GammaStar(k)=sup.GammaStar;
H.ASRGain(k)=sup.gain; H.criticalIndex(k)=sup.criticalIndex;
H.gammaActual(k)=gammaActual; H.demandScale(k)=cmd.demandScale;
H.energy(k)=energy; H.modeCode(k)=modeToCode(sup.mode);
H.degradedFlag(k)=sup.degradedFlag; H.solverStatus(k)=cmd.solverStatus;
H.constraintViolation(k)=aux.constraintViolation;
H.Fz(:,k)=x.Fz; H.FzTarget(:,k)=sup.FzTarget; H.Fx(:,k)=x.Fx;
H.Fy(:,k)=x.Fy; H.Tcmd(:,k)=cmd.Tcmd; H.Fa(:,k)=x.Faact;
H.rho(:,k)=aux.rho; H.lambdaTrue(:,k)=lambdaTrue;
H.lambdaEst(:,k)=lambdaEst; H.mu(:,k)=mu; H.gDemand(:,k)=demand.g;
H.sensitivity(:,k)=sup.sensitivity;
nr = min(numel(sup.gammaRide),NpMax);
ns = min(numel(sup.gammaStar),NpMax);
H.gammaRideH(1:nr,k)=sup.gammaRide(1:nr);
H.gammaStarH(1:ns,k)=sup.gammaStar(1:ns);
end

function code = modeToCode(mode)
switch string(mode)
    case "certified", code = 1;
    case "shape-recoverable", code = 2;
    case "uncertified", code = 3;
    case "joint-MPC", code = 4;
    otherwise, code = 0;
end
end

