function cfg = defaultConfig()
%DEFAULTCONFIG Nominal model, controller, and experiment parameters.

cfg.meta.parameterProfile = "literature_benchmark_v1";
cfg.meta.coreVehicleSource = "Hu et al., Applied Sciences, 2022, doi:10.3390/app122211768";
cfg.meta.tyreSource = "Wang et al., Energies, 2022, doi:10.3390/en15155728";
cfg.meta.motorSource = "Zhang et al., Sensors, 2023, doi:10.3390/s23073505";
cfg.meta.motorDynamicsSource = "Nah and Yim, Electronics, 2020, doi:10.3390/electronics9111934";
cfg.meta.suspensionActuatorSource = "Kim and Yim, Actuators, 2024, doi:10.3390/act13120520";
cfg.meta.uncertaintySource = "Previati et al., World Electric Vehicle Journal, 2024, doi:10.3390/wevj15060266";

cfg.sim.Ts = 0.01;
cfg.sim.dtPlant = 0.002;
cfg.sim.g = 9.81;
cfg.sim.seed = 20260722;

% Core 10-DOF vehicle parameters reported by Hu et al. (2022).
cfg.vehicle.m = 1395;
cfg.vehicle.ms = 1245;
cfg.vehicle.Iz = 1356;
cfg.vehicle.Ix = 480;
cfg.vehicle.Iy = 1356;
cfg.vehicle.a = 1.080;
cfg.vehicle.b = 1.620;
cfg.vehicle.L = cfg.vehicle.a + cfg.vehicle.b;
cfg.vehicle.tf = 1.535;
cfg.vehicle.tr = 1.535;
cfg.vehicle.hCG = mean([0.453,0.601]); % Midpoint of the measured BEV range in R6.
cfg.vehicle.hRoll = 0.254;   % Sprung-mass CG above roll centre, Hu et al. (2022).
cfg.vehicle.hPitch = 0.104;  % Sprung-mass CG above pitch centre, Hu et al. (2022).
cfg.vehicle.Rw = 0.298;
cfg.vehicle.Iw = 2.2;
cfg.vehicle.frontalArea = 2.01;
cfg.vehicle.Cd = 0.29;
cfg.vehicle.Crr = 0.009;
cfg.vehicle.xw = [cfg.vehicle.a; cfg.vehicle.a; -cfg.vehicle.b; -cfg.vehicle.b];
cfg.vehicle.yw = [cfg.vehicle.tf/2; -cfg.vehicle.tf/2; cfg.vehicle.tr/2; -cfg.vehicle.tr/2];

cfg.vehicle.ks = [27358;27358;19600;19600];
cfg.vehicle.cs = [1695;1695;1965;1965];
cfg.vehicle.Kphi = sum(cfg.vehicle.ks.*cfg.vehicle.yw.^2);
cfg.vehicle.Cphi = sum(cfg.vehicle.cs.*cfg.vehicle.yw.^2);
cfg.vehicle.Ktheta = sum(cfg.vehicle.ks.*cfg.vehicle.xw.^2);
cfg.vehicle.Ctheta = sum(cfg.vehicle.cs.*cfg.vehicle.xw.^2);
cfg.vehicle.Kheave = sum(cfg.vehicle.ks);
cfg.vehicle.Cheave = sum(cfg.vehicle.cs);
cfg.vehicle.muUnsprung = (cfg.vehicle.m-cfg.vehicle.ms)/4 * ones(4,1);
cfg.vehicle.kt = [87e3;87e3;92e3;92e3];
cfg.vehicle.ct = 1200;

Fzf = cfg.vehicle.m * cfg.sim.g * cfg.vehicle.b / (2*cfg.vehicle.L);
Fzr = cfg.vehicle.m * cfg.sim.g * cfg.vehicle.a / (2*cfg.vehicle.L);
cfg.vehicle.Fz0 = [Fzf; Fzf; Fzr; Fzr];

% Equivalent linear tyre stiffness reported by Wang et al. (2022).
cfg.tyre.CalphaFront = 66e3;
cfg.tyre.CalphaRear = 66e3;
cfg.tyre.relaxation = 0.06;
cfg.tyre.loadSensitivity = 0.90;
cfg.tyre.FzMinRatio = 0.05;
cfg.tyre.tightening = 0.94;

cfg.motor.Tmax = 600; % Peak in-wheel-motor torque, Zhang et al. (2023).
cfg.motor.tau = 0.050; % First-order 4WID actuator time constant, Nah and Yim (2020).
cfg.motor.lambdaMin = 0.05;

cfg.susp.Fmax = 4000;
cfg.susp.bandwidth = 10;
cfg.susp.tau = 1/(2*pi*cfg.susp.bandwidth);
cfg.susp.FrateMax = cfg.susp.Fmax/cfg.susp.tau;
cfg.susp.strokeMax = 0.08;
cfg.susp.loadGain = 0.72;
cfg.susp.loadShiftMax = 0.90*cfg.susp.loadGain*cfg.susp.Fmax;
cfg.susp.loadShiftRate = cfg.susp.loadGain*cfg.susp.FrateMax;

cfg.scale.Fref = cfg.vehicle.m * cfg.sim.g;
cfg.scale.Mzref = cfg.vehicle.m * cfg.sim.g * mean([cfg.vehicle.tf,cfg.vehicle.tr]) / 2;
cfg.scale.Mphiref = cfg.vehicle.ms * cfg.sim.g * cfg.vehicle.hRoll;
cfg.scale.Sg = diag(1 ./ [cfg.scale.Fref, cfg.scale.Mzref, cfg.scale.Mphiref]);

cfg.control.Np = 15;
cfg.control.Nc = 5;
cfg.control.deltaGamma = 0.10;
cfg.control.epsilonH = 0.02;
cfg.control.epsilonG = 0.02;
cfg.control.epsilonEta = 0.01;
cfg.control.dwellSamples = 3;
cfg.control.gammaCap = 3.0;
cfg.control.fdLoad = 80;
cfg.control.trustInitial = 1.0;
cfg.control.trustMin = 0.125;
cfg.control.maxShapingIterations = 3;
cfg.control.supervisorStride = 1;
cfg.control.Kspeed = 0.9 * cfg.vehicle.m;
cfg.control.Kyaw = 5200;
cfg.control.Kbeta = 3600;
cfg.control.Kroll = 4.5e4;
cfg.control.KrollRate = 6e3;
cfg.control.Kpath = 0.045;
cfg.control.Kheading = 0.70;
cfg.control.deltaMax = deg2rad(9);
cfg.control.FxDemandMax = 0.65 * cfg.vehicle.m * cfg.sim.g;
cfg.control.MzDemandMax = 0.30 * cfg.scale.Mzref;
cfg.control.MphiDemandMax = 0.45 * cfg.scale.Mphiref;
cfg.control.muUncertainty = 0.04;
cfg.control.lambdaUncertainty = 0.03;

cfg.allocator.wFx = 1.0;
cfg.allocator.wFa = 0.15;
cfg.allocator.wLoad = 4.0;
cfg.allocator.wDelta = 0.15;
cfg.allocator.reserveTolerance = 1e-5;

cfg.experiment.mode = "quick";
cfg.experiment.methods = ["Healthy","No_FTC","Torque_FTC","RT_FTC", ...
    "Parallel_ICC","J_MPC","R_ASR","P_ASR"];
cfg.experiment.boundaryMethods = ["Torque_FTC","RT_FTC","J_MPC","R_ASR","P_ASR"];
cfg.experiment.mcMethods = ["Torque_FTC","RT_FTC","Parallel_ICC","J_MPC","R_ASR","P_ASR"];
cfg.experiment.lambdaGridPaper = 0:0.10:1.00;
cfg.experiment.muGridPaper = 0.25:0.05:0.80;
cfg.experiment.delayGridPaper = [0,0.05,0.10,0.15,0.20];
cfg.experiment.biasGridPaper = [-0.15,0,0.15];
cfg.experiment.forceScalePaper = [0.60,0.80,1.00,1.20];
cfg.experiment.tauScalePaper = [0.75,1.00,1.50,2.00];
cfg.experiment.mcCountPaper = 200;
cfg.experiment.delaySeedsPaper = 50;
cfg.experiment.mismatchSeedsPaper = 40;

cfg.output.saveFigures = false;
cfg.output.saveMat = true;
cfg.output.directory = fullfile(pwd, 'results');
cfg.output.figureDpi = 300;
cfg.output.fontName = 'Times New Roman';
cfg.output.fontSize = 9;
end
