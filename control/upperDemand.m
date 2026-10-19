function demand = upperDemand(x, ref, cfg)
%UPPERDEMAND Vehicle-level longitudinal, yaw, and roll demands.

beta = atan2(x.vy,max(x.vx,1));
Fx = cfg.control.Kspeed*(ref.v-x.vx);
Mz = cfg.control.Kyaw*(ref.r-x.r)-cfg.control.Kbeta*beta;
Mphi = -cfg.control.Kroll*x.phi-cfg.control.KrollRate*x.phidot;

Fx = min(max(Fx,-cfg.control.FxDemandMax),cfg.control.FxDemandMax);
Mz = min(max(Mz,-cfg.control.MzDemandMax),cfg.control.MzDemandMax);
Mphi = min(max(Mphi,-cfg.control.MphiDemandMax),cfg.control.MphiDemandMax);

demand.g = [Fx;Mz;Mphi];
demand.gT = [Fx;Mz];
demand.beta = beta;
demand.rError = ref.r-x.r;
end

