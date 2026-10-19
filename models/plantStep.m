function [x, aux] = plantStep(x, cmd, ref, lambdaTrue, muRoad, scn, cfg)
%PLANTSTEP Independent nonlinear Plant A with actuator and load dynamics.

dt = cfg.sim.dtPlant;
nSub = max(1,round(cfg.sim.Ts/dt));
dt = cfg.sim.Ts/nSub;

m = cfg.vehicle.m*scn.parameterScale.mass;
Iz = cfg.vehicle.Iz*scn.parameterScale.Iz;
ms = cfg.vehicle.ms;
g = cfg.sim.g;
hCG = cfg.vehicle.hCG*scn.parameterScale.hCG;
Fz0Case = cfg.vehicle.Fz0*scn.parameterScale.mass;
FzMinCase = scn.FzMin*scn.parameterScale.mass;
P = eye(4)-ones(4)/4;
FaMaxReal = cfg.susp.Fmax*scn.forceScale;
tauAReal = cfg.susp.tau*scn.tauScale;

for h = 1:nSub
    Ttarget = lambdaTrue .* cmd.Tcmd;
    x.Tact = x.Tact + dt*(Ttarget-x.Tact)/cfg.motor.tau;
    Fatarget = min(max(cmd.Facmd,-FaMaxReal),FaMaxReal);
    x.Faact = x.Faact + dt*(Fatarget-x.Faact)/tauAReal;

    zBodyCorner = x.zs + cfg.vehicle.xw*x.theta + cfg.vehicle.yw*x.phi;
    vBodyCorner = x.zsdot + cfg.vehicle.xw*x.thetadot + cfg.vehicle.yw*x.phidot;
    x.stroke = zBodyCorner-x.zu;
    strokeRate = vBodyCorner-x.zudot;
    suspensionForce = -cfg.vehicle.ks.*x.stroke ...
        -cfg.vehicle.cs.*strokeRate+x.Faact;

    dFLong = m*hCG*x.ax/cfg.vehicle.L;
    FzLong = [-0.5;-0.5;0.5;0.5]*dFLong;
    dFf = (m*cfg.vehicle.b/cfg.vehicle.L)*hCG*x.ay/cfg.vehicle.tf;
    dFr = (m*cfg.vehicle.a/cfg.vehicle.L)*hCG*x.ay/cfg.vehicle.tr;
    FzLat = [-0.5*dFf;0.5*dFf;-0.5*dFr;0.5*dFr];
    FzActive = cfg.susp.loadGain*P*x.Faact;
    roadLoad = cfg.vehicle.kt.*(ref.road-x.zu) - cfg.vehicle.ct*x.zudot;
    x.Fz = Fz0Case + FzLong + FzLat + FzActive + roadLoad;
    x.Fz = max(x.Fz,FzMinCase);

    deltaW = [ref.delta;ref.delta;0;0];
    alpha = zeros(4,1);
    alpha(1:2) = deltaW(1:2)-atan2(x.vy+cfg.vehicle.a*x.r,max(x.vx,1));
    alpha(3:4) = -atan2(x.vy-cfg.vehicle.b*x.r,max(x.vx,1));
    C = [cfg.tyre.CalphaFront;cfg.tyre.CalphaFront; ...
        cfg.tyre.CalphaRear;cfg.tyre.CalphaRear];
    FyPure = C.*alpha.*(x.Fz./Fz0Case).^cfg.tyre.loadSensitivity;

    FxDemand = x.Tact/cfg.vehicle.Rw;
    FxLimit = muRoad.*x.Fz;
    x.Fx = min(max(FxDemand,-FxLimit),FxLimit);
    FyLimit = sqrt(max((muRoad.*x.Fz).^2-x.Fx.^2,0));
    x.Fy = min(max(FyPure,-FyLimit),FyLimit);

    FxBody = x.Fx.*cos(deltaW)-x.Fy.*sin(deltaW);
    FyBody = x.Fx.*sin(deltaW)+x.Fy.*cos(deltaW);
    Fres = 0.5*1.225*cfg.vehicle.Cd*cfg.vehicle.frontalArea*x.vx*abs(x.vx) ...
        +cfg.vehicle.Crr*m*g;
    ax = sum(FxBody)/m + x.r*x.vy - sign(x.vx)*Fres/m;
    ay = sum(FyBody)/m - x.r*x.vx;
    rdot = sum(cfg.vehicle.xw.*FyBody-cfg.vehicle.yw.*FxBody)/Iz;

    phiddot = (ms*cfg.vehicle.hRoll*(ay+x.vx*x.r) + ms*g*cfg.vehicle.hRoll*x.phi ...
        +cfg.vehicle.yw.'*suspensionForce)/cfg.vehicle.Ix;
    thetaddot = (-ms*cfg.vehicle.hPitch*ax ...
        +cfg.vehicle.xw.'*suspensionForce)/cfg.vehicle.Iy;
    zsddot = sum(suspensionForce)/ms;

    zuAcc = (roadLoad-suspensionForce)./cfg.vehicle.muUnsprung;
    omegaDot = (x.Tact-cfg.vehicle.Rw*x.Fx)/cfg.vehicle.Iw;

    x.vx = max(0.5,x.vx+dt*ax);
    x.vy = x.vy+dt*ay;
    x.r = x.r+dt*rdot;
    x.psi = x.psi+dt*x.r;
    x.X = x.X+dt*(x.vx*cos(x.psi)-x.vy*sin(x.psi));
    x.Y = x.Y+dt*(x.vx*sin(x.psi)+x.vy*cos(x.psi));
    x.phidot = x.phidot+dt*phiddot;
    x.phi = x.phi+dt*x.phidot;
    x.thetadot = x.thetadot+dt*thetaddot;
    x.theta = x.theta+dt*x.thetadot;
    x.zsdot = x.zsdot+dt*zsddot;
    x.zs = x.zs+dt*x.zsdot;
    x.zudot = x.zudot+dt*zuAcc;
    x.zu = x.zu+dt*x.zudot;
    x.omega = max(0,x.omega+dt*omegaDot);
    x.ax = ax;
    x.ay = ay;
    x.stroke = x.zs + cfg.vehicle.xw*x.theta + cfg.vehicle.yw*x.phi-x.zu;
end

rho = sqrt((x.Fx./max(muRoad.*x.Fz,1)).^2 + ...
    (x.Fy./max(muRoad.*x.Fz,1)).^2);
aux.rho = rho;
aux.rhoMax = max(rho);
aux.beta = atan2(x.vy,max(x.vx,1));
vBodyCorner = x.zsdot+cfg.vehicle.xw*x.thetadot+cfg.vehicle.yw*x.phidot;
aux.Pa = sum(abs(x.Faact.*(vBodyCorner-x.zudot)));
aux.constraintViolation = any(x.Fz < FzMinCase-1e-6) || ...
    any(abs(x.stroke)>cfg.susp.strokeMax) || any(rho>1.001);
end
