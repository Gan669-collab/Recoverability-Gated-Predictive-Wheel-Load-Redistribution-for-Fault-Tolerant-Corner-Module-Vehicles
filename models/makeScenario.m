function scn = makeScenario(name, cfg)
%MAKESCENARIO Define deterministic manoeuvres used in E2-E4.

name = upper(string(name));
scn.name = name;
scn.faultWheel = 1;
scn.faultTime = 2.0;
scn.lambdaFault = 0.30;
scn.muLeft = 0.40;
scn.muRight = 0.40;
scn.speed0 = 100/3.6;
scn.duration = 7.0;
scn.healthDelay = 0;
scn.healthBias = 0;
scn.forceScale = 1;
scn.tauScale = 1;
scn.parameterScale.mass = 1;
scn.parameterScale.Iz = 1;
scn.parameterScale.hCG = 1;
scn.frictionError = 0;
scn.roadAmplitude = 0.003;

switch name
    case "DLC"
        scn.description = "ISO-like double lane change";
        scn.speed0 = 100/3.6;
        scn.duration = 7.0;
        scn.faultWheel = 1;
        scn.lambdaFault = 0.30;
        scn.faultTime = 1.8;
        scn.muLeft = 0.40;
        scn.muRight = 0.40;
    case "SWD"
        scn.description = "Sine with dwell";
        scn.speed0 = 80/3.6;
        scn.duration = 6.5;
        scn.faultWheel = 3;
        scn.lambdaFault = 0;
        scn.faultTime = 1.5;
        scn.muLeft = 0.35;
        scn.muRight = 0.35;
    case "SPLIT"
        scn.description = "Split-mu braking and steering";
        scn.speed0 = 80/3.6;
        scn.duration = 6.0;
        scn.faultWheel = 1;
        scn.lambdaFault = 0.20;
        scn.faultTime = 1.2;
        scn.muLeft = 0.80;
        scn.muRight = 0.30;
    otherwise
        error('Unknown scenario: %s', name);
end

scn.FzMin = cfg.tyre.FzMinRatio * cfg.vehicle.Fz0;
end
