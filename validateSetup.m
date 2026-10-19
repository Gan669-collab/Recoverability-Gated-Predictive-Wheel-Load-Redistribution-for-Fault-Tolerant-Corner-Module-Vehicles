function report = validateSetup()
%VALIDATESETUP Check paths, required solvers, and the default configuration.

root = fileparts(mfilename('fullpath'));
addpath(genpath(root));

report.matlabRelease = version('-release');
report.linprogAvailable = exist('linprog','file') == 2;
report.quadprogAvailable = exist('quadprog','file') == 2;
report.entryPointAvailable = exist('simulateCase','file') == 2;

cfg = defaultConfig();
P = parameterProvenance(cfg);
report.parameterProfile = cfg.meta.parameterProfile;
report.parameterCount = height(P);
assert(cfg.sim.Ts > 0 && cfg.sim.dtPlant > 0,'Sampling times must be positive.');
assert(cfg.control.Np >= 1,'The prediction horizon must be positive.');
assert(all(cfg.vehicle.Fz0 > 0),'Nominal wheel loads must be positive.');
assert(numel(cfg.experiment.methods) == 8,'Eight comparison methods are expected.');
assert(report.parameterCount == 30,'The parameter provenance table is incomplete.');

fprintf('MATLAB release: %s\n',report.matlabRelease);
fprintf('Parameter profile: %s (%d traced entries)\n', ...
    char(report.parameterProfile),report.parameterCount);
fprintf('linprog: %s\n',statusText(report.linprogAvailable));
fprintf('quadprog: %s\n',statusText(report.quadprogAvailable));
fprintf('Project entry points: %s\n',statusText(report.entryPointAvailable));

if ~(report.linprogAvailable && report.quadprogAvailable)
    warning(['Optimization Toolbox was not detected. Conservative algebraic ', ...
        'fallbacks will be used, so these results must not be used as paper data.']);
end
end

function txt = statusText(flag)
if flag
    txt = 'available';
else
    txt = 'missing';
end
end
