function R = run_all(mode, saveFigures)
%RUN_ALL Execute the P-ASR experiment suite and draw Figures 3-12.
%   R = RUN_ALL("quick", false) runs a reduced verification suite.
%   R = RUN_ALL("paper", true) uses the experiment counts in the paper.

if nargin < 1 || isempty(mode)
    mode = "quick";
end
if nargin < 2
    saveFigures = false;
end

root = fileparts(mfilename('fullpath'));
addpath(genpath(root));

cfg = defaultConfig();
cfg.experiment.mode = string(mode);
cfg.output.saveFigures = logical(saveFigures);
cfg.output.directory = fullfile(root, 'results');
if ~exist(cfg.output.directory, 'dir')
    mkdir(cfg.output.directory);
end

fprintf('P-ASR simulation suite: %s mode\n', cfg.experiment.mode);
tStart = tic;
R = runExperimentSuite(cfg);
R.requestedConfig = cfg;
R.elapsedSeconds = toc(tStart);

plotFigures3to12(R, R.config);

if cfg.output.saveMat
    stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    save(fullfile(cfg.output.directory, ['P_ASR_results_' stamp '.mat']), ...
        'R', '-v7.3');
end

fprintf('Completed in %.1f s. Results are stored in R.\n', R.elapsedSeconds);
end
