function scn = sampleUncertainScenario(baseScn,seed,cfg)
%SAMPLEUNCERTAINSCENARIO Common-seed uncertainty draw for paired comparisons.

rng(seed,'twister');
scn = baseScn;
scn.parameterScale.mass = 0.90+0.20*rand;
scn.parameterScale.Iz = 0.85+0.30*rand;
scn.parameterScale.hCG = 0.90+0.20*rand;
scn.frictionError = -0.20+0.40*rand;
scn.healthBias = -0.08+0.16*rand;
scn.faultTime = max(0.8,baseScn.faultTime-0.20+0.40*rand);
scn.lambdaFault = min(max(baseScn.lambdaFault-0.10+0.20*rand,0),1);
scn.speed0 = baseScn.speed0*(0.95+0.10*rand);
scn.tauScale = 0.85+0.35*rand;
scn.forceScale = 0.90+0.20*rand;
scn.roadAmplitude = baseScn.roadAmplitude*(0.5+rand);
scn.FzMin = cfg.tyre.FzMinRatio*cfg.vehicle.Fz0;
end
