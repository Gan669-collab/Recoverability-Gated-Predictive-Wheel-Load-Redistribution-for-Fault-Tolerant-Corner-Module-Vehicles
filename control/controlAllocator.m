function [cmd,allocState] = controlAllocator(method,demand,sup,x,ref,lambdaEst,muEst,allocState,cfg)
%CONTROLALLOCATOR Demand-priority allocation with secondary load shaping.

method = upper(string(method));
c = [cos(ref.delta);cos(ref.delta);1;1];
b = cfg.vehicle.xw.*[sin(ref.delta);sin(ref.delta);0;0]-cfg.vehicle.yw.*c;
BT = [c.';b.'];
B = [BT,zeros(2,4);zeros(1,4),cfg.vehicle.yw.'];

if method == "HEALTHY"
    lambdaUse = ones(4,1);
else
    lambdaUse = lambdaEst;
end

motorCap = lambdaUse*cfg.motor.Tmax/cfg.vehicle.Rw;
frictionCap = sqrt(max((muEst.*x.Fz).^2-x.Fy.^2,0));
FxCap = max(0,min(motorCap,frictionCap)*cfg.tyre.tightening);
maxDeltaFa = cfg.susp.FrateMax*cfg.sim.Ts;
FaLower = max(-cfg.susp.Fmax*ones(4,1),allocState.lastFa-maxDeltaFa);
FaUpper = min( cfg.susp.Fmax*ones(4,1),allocState.lastFa+maxDeltaFa);

baselineFa = cfg.vehicle.yw*demand.g(3)/max(sum(cfg.vehicle.yw.^2),1);
baselineFa = min(max(baselineFa,-cfg.susp.Fmax),cfg.susp.Fmax);

if any(method == ["NO_FTC","HEALTHY"])
    Fx = pinv(BT)*demand.gT;
    Fx = min(max(Fx,-cfg.motor.Tmax/cfg.vehicle.Rw),cfg.motor.Tmax/cfg.vehicle.Rw);
    Fa = baselineFa;
    if method == "NO_FTC"
        Tcmd = cfg.vehicle.Rw*Fx;
    else
        Tcmd = cfg.vehicle.Rw*Fx;
    end
    solverStatus = 1;
    demandScale = 1;
elseif any(method == ["TORQUE_FTC","RT_FTC","PARALLEL_ICC","A0"])
    g2 = demand.gT;
    muRobust = muEst;
    lambdaRobust = lambdaUse;
    if any(method == ["RT_FTC"])
        muRobust = max(muEst-cfg.control.muUncertainty,0.05);
        lambdaRobust = max(lambdaUse-cfg.control.lambdaUncertainty,0);
    end
    [gamma,~,~] = directionalMargin([g2;0],x.Fz,x.Fy,lambdaRobust,muRobust, ...
        ref.delta,cfg,false);
    demandScale = min(1,gamma);
    W = diag(1./max(FxCap,100).^2);
    [Fx,solverStatus] = solveBoundedQP(2*W,zeros(4,1),BT,demandScale*g2, ...
        -FxCap,FxCap,pinv(BT)*(demandScale*g2));
    if method == "PARALLEL_ICC"
        Fa = baselineFa;
    else
        Fa = zeros(4,1);
    end
    Tcmd = forceToTorque(Fx,lambdaUse,cfg);
else
    lambdaRobust = max(lambdaUse-cfg.control.lambdaUncertainty,0);
    muRobust = max(muEst-cfg.control.muUncertainty,0.05);
    [gamma,~,~] = directionalMargin(demand.g,x.Fz,x.Fy,lambdaRobust,muRobust, ...
        ref.delta,cfg,true);
    demandScale = min(1,gamma);

    P = eye(4)-ones(4)/4;
    dFTarget = sup.FzTarget-x.Fz;
    FaTarget = pinv(cfg.susp.loadGain*P)*dFTarget;
    FaTarget = min(max(FaTarget,-cfg.susp.Fmax),cfg.susp.Fmax);

    Hfx = diag(1./max(FxCap,100).^2);
    Hfa = cfg.allocator.wFa*eye(4)+cfg.allocator.wLoad*(cfg.susp.loadGain*P)'*(cfg.susp.loadGain*P);
    H = blkdiag(Hfx,Hfa)+cfg.allocator.wDelta*eye(8);
    f = -cfg.allocator.wLoad*[zeros(4,1);(cfg.susp.loadGain*P)'*dFTarget] ...
        -cfg.allocator.wDelta*allocState.lastU;
    lb = [-FxCap;FaLower];
    ub = [ FxCap;FaUpper];
    u0 = [zeros(4,1);min(max(FaTarget,FaLower),FaUpper)];
    [u,solverStatus] = solveBoundedQP(2*H,2*f,B,demandScale*demand.g,lb,ub,u0);
    Fx = u(1:4);
    Fa = u(5:8);
    Tcmd = forceToTorque(Fx,lambdaUse,cfg);
end

Fa = allocState.lastFa + min(max(Fa-allocState.lastFa,-maxDeltaFa),maxDeltaFa);
Fa = min(max(Fa,-cfg.susp.Fmax),cfg.susp.Fmax);
Tcmd = min(max(Tcmd,-cfg.motor.Tmax),cfg.motor.Tmax);

cmd.Tcmd = Tcmd;
cmd.Facmd = Fa;
cmd.FxDesired = Fx;
cmd.demandScale = demandScale;
cmd.solverStatus = solverStatus;
allocState.lastU = [Fx;Fa];
allocState.lastFa = Fa;
allocState.lastT = Tcmd;
end

function T = forceToTorque(Fx,lambda,cfg)
T = zeros(4,1);
active = lambda >= cfg.motor.lambdaMin;
T(active) = cfg.vehicle.Rw*Fx(active)./max(lambda(active),cfg.motor.lambdaMin);
end

function [u,status] = solveBoundedQP(H,f,Aeq,beq,lb,ub,u0)
status = -1;
u = [];
if exist('quadprog','file') == 2
    try
        opts = optimoptions('quadprog','Display','none','Algorithm','interior-point-convex');
        [u,~,status] = quadprog(H,f,[],[],Aeq,beq,lb,ub,u0,opts);
    catch
        u = [];
    end
end
if isempty(u) || status <= 0
    u = pinv(Aeq)*beq;
    u = min(max(u,lb),ub);
    % Project once onto the equality manifold before reapplying bounds.
    u = u + pinv(Aeq)*(beq-Aeq*u);
    u = min(max(u,lb),ub);
    status = 0;
end
end
