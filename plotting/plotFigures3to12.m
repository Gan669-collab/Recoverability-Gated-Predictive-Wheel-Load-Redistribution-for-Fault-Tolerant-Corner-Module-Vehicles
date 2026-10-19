function plotFigures3to12(R,cfg)
%PLOTFIGURES3TO12 Generate the ten result figures declared in the manuscript.

plotFigure3(R,cfg);
plotFigure4(R,cfg);
plotFigure5(R,cfg);
plotFigure6(R,cfg);
plotFigure7(R,cfg);
plotFigure8(R,cfg);
plotFigure9(R,cfg);
plotFigure10(R,cfg);
plotFigure11(R,cfg);
plotFigure12(R,cfg);
end

function plotFigure3(R,cfg)
out = R.deterministic.DLC.P_ASR;
H = out.history;
k = find(H.t>=out.scenario.faultTime+0.20,1,'first');
fig = newFigure('Figure 3 - Authority reconstruction');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile;
theta = linspace(0,2*pi,81);
healthy = boundary2D(theta,H.Fz(:,k),H.Fy(:,k),ones(4,1),H.mu(:,k),H.delta(k),cfg);
degraded = boundary2D(theta,H.Fz(:,k),H.Fy(:,k),H.lambdaTrue(:,k),H.mu(:,k),H.delta(k),cfg);
plot(healthy(1,:)/1000,healthy(2,:)/1000,'k--','LineWidth',1.2); hold on;
plot(degraded(1,:)/1000,degraded(2,:)/1000,'r-','LineWidth',1.4);
xlabel('F_x (kN)'); ylabel('M_z (kN m)'); title('(a) Instantaneous boundary'); grid on;
legend('Healthy','Degraded','Location','best');

nexttile;
NpAvailable = size(H.gammaStarH,1);
criticalIndex = min(NpAvailable,max(1,round(H.criticalIndex(k))));
idx = unique([1,criticalIndex,NpAvailable]);
colors = lines(numel(idx));
for ii=1:numel(idx)
    gamma = H.gammaStarH(idx(ii),k);
    if isnan(gamma), gamma = H.GammaStar(k); end
    plot(gamma*degraded(1,:)/1000,gamma*degraded(2,:)/1000, ...
        'Color',colors(ii,:),'LineWidth',1.1); hold on;
end
g = H.gDemand(1:2,k);
plot([0,g(1)]/1000,[0,g(2)]/1000,'ko-','LineWidth',1.2);
xlabel('F_x (kN)'); ylabel('M_z (kN m)'); title('(b) Reachable horizon sections'); grid on;
labels = [compose('j=%d',idx),"Demand ray"];
legend(labels,'Location','best');

nexttile;
plot(H.t,H.GammaRide,'LineWidth',1.2); hold on;
plot(H.t,H.GammaStar,'LineWidth',1.2); yline(1+cfg.control.deltaGamma,'k--');
stairs(H.t,0.15*H.modeCode,'Color',[0.4 0.4 0.4]);
xlabel('Time (s)'); ylabel('Directional margin'); title('(c) Two-envelope gate'); grid on;
legend('\Gamma^{ride}','\Gamma^*','Threshold','Mode code','Location','best');

nexttile;
yyaxis left; bar(categorical({'FL','FR','RL','RR'}),H.FzTarget(:,k)-H.Fz(:,k));
ylabel('\Delta F_z (N)');
yyaxis right; plot(categorical({'FL','FR','RL','RR'}),H.sensitivity(:,k),'o-','LineWidth',1.2);
ylabel('Load effectiveness'); title('(d) Directional load update'); grid on;
finishFigure(fig,3,cfg);
end

function plotFigure4(R,cfg)
A = R.audit;
fig = newFigure('Figure 4 - Certificate audit');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; scatter(A.predicted,A.realized,18,A.time,'filled'); hold on;
lims=[min([A.predicted,A.realized]),max([A.predicted,A.realized])]; plot(lims,lims,'k--');
xlabel('Predicted reach'); ylabel('Nonlinear realized reach'); title('(a) Ray-reach audit'); grid on; colorbar;
nexttile; histogram(A.gap,20,'FaceColor',[0.2 0.45 0.75]);
xlabel('Conservatism gap'); ylabel('Count'); title('(b) Gap distribution'); grid on;
nexttile; plot(A.time,A.predicted,'LineWidth',1.1); hold on; plot(A.time,A.realized,'LineWidth',1.1);
yline(1,'k--'); xlabel('Time (s)'); ylabel('Margin'); title('(c) Predicted and realized loss'); grid on;
legend('Predicted','Realized','Unity','Location','best');
nexttile; bar([sum(A.falseSafe),sum(A.falseRecoverable)]);
set(gca,'XTickLabel',{'False-safe','False-recoverable'}); ylabel('Sample count');
title('(d) Audit events'); grid on;
finishFigure(fig,4,cfg);
end

function plotFigure5(R,cfg)
methods = ["No_FTC","Torque_FTC","RT_FTC","J_MPC","R_ASR","P_ASR"];
fig = newFigure('Figure 5 - Double lane change');
tiledlayout(3,3,'TileSpacing','compact','Padding','compact');
nexttile;
for m=methods
    H=R.deterministic.DLC.(char(m)).history; plot(H.X,H.Y,'LineWidth',1.0); hold on;
end
xlabel('X (m)'); ylabel('Y (m)'); title('(a) Vehicle trajectories'); grid on;
legend(methodLabel(methods),'Location','best','FontSize',7);
nexttile; compareTrace(R.deterministic.DLC,methods,'Y',1,'Lateral position (m)','(b) Lateral response');
nexttile; compareTrace(R.deterministic.DLC,methods,'r',180/pi,'Yaw rate (deg/s)','(c) Yaw rate');
nexttile; compareTrace(R.deterministic.DLC,methods,'beta',180/pi,'Sideslip (deg)','(d) Sideslip angle');
nexttile; compareTrace(R.deterministic.DLC,methods,'gammaActual',1,'Realized PDAR','(e) Directional reserve'); yline(1,'k--');
nexttile; H=R.deterministic.DLC.P_ASR.history; plot(H.t,H.Tcmd'/1000); xlabel('Time (s)'); ylabel('Torque (kN m)'); title('(f) P-ASR wheel torques'); grid on;
nexttile; plot(H.t,H.Fz'/1000); xlabel('Time (s)'); ylabel('F_z (kN)'); title('(g) P-ASR normal loads'); grid on;
nexttile; plot(H.t,H.rho'); yline(1,'k--'); xlabel('Time (s)'); ylabel('\rho_i'); title('(h) Tyre utilization'); grid on;
nexttile; vals=zeros(numel(methods),5);
for i=1:numel(methods)
    M=R.deterministic.DLC.(char(methods(i))).metrics;
    vals(i,:)=[M.lateralRMSE,M.yawRMSE,M.peakSideslipDeg,M.peakRho,M.recoveryTime];
end
bar(vals); set(gca,'XTickLabel',methodLabel(methods),'XTickLabelRotation',25);
ylabel('Metric value'); title('(i) Motion and utilization metrics'); grid on;
finishFigure(fig,5,cfg);
end

function plotFigure6(R,cfg)
methods=["Torque_FTC","J_MPC","R_ASR","P_ASR"];
fig=newFigure('Figure 6 - Sine with dwell'); tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
nexttile; compareTrace(R.deterministic.SWD,methods,'r',180/pi,'Yaw rate (deg/s)','(a) Yaw response');
nexttile; compareTrace(R.deterministic.SWD,methods,'beta',180/pi,'Sideslip (deg)','(b) Sideslip angle');
nexttile; compareTrace(R.deterministic.SWD,methods,'phi',180/pi,'Roll angle (deg)','(c) Roll angle');
nexttile; H=R.deterministic.SWD.P_ASR.history; plot(H.t,H.Fz'/1000); xlabel('Time (s)'); ylabel('F_z (kN)'); title('(d) Wheel loads'); grid on;
nexttile; plot(H.t,H.rho'); yline(1,'k--'); xlabel('Time (s)'); ylabel('\rho_i'); title('(e) Tyre utilization'); grid on;
nexttile; for m=methods, H=R.deterministic.SWD.(char(m)).history; plot(H.t,H.energy/1000,'LineWidth',1.1); hold on; end
xlabel('Time (s)'); ylabel('Energy (kJ)'); title('(f) Suspension energy'); grid on; legend(methodLabel(methods),'Location','best');
finishFigure(fig,6,cfg);
end

function plotFigure7(R,cfg)
methods=["Torque_FTC","RT_FTC","J_MPC","R_ASR","P_ASR"];
fig=newFigure('Figure 7 - Split friction'); tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
nexttile; compareTrace(R.deterministic.SPLIT,methods,'vx',3.6,'Speed (km/h)','(a) Vehicle speed');
nexttile; compareTrace(R.deterministic.SPLIT,methods,'r',180/pi,'Yaw rate (deg/s)','(b) Yaw response');
nexttile; compareTrace(R.deterministic.SPLIT,methods,'Y',1,'Lateral position (m)','(c) Path response');
nexttile; H=R.deterministic.SPLIT.P_ASR.history; plot(H.t,H.Fx'/1000); xlabel('Time (s)'); ylabel('F_x (kN)'); title('(d) Longitudinal forces'); grid on;
nexttile; plot(H.t,H.rho'); yline(1,'k--'); xlabel('Time (s)'); ylabel('\rho_i'); title('(e) Utilization'); grid on;
nexttile; plot(H.t,H.GammaRide,'LineWidth',1.1); hold on; plot(H.t,H.gammaActual,'LineWidth',1.1); yline(1,'k--');
xlabel('Time (s)'); ylabel('Margin'); title('(f) Predicted/realized critical time'); grid on; legend('Predicted','Realized','Unity');
finishFigure(fig,7,cfg);
end

function plotFigure8(R,cfg)
methods=R.boundary.methods;
fig=newFigure('Figure 8 - Recoverability boundary'); tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
for i=1:numel(methods)
    nexttile; A=R.boundary.recoverable.(char(methods(i)));
    imagesc(R.boundary.lambda,R.boundary.mu,A); axis xy; caxis([0 1]);
    xlabel('Retained effectiveness \lambda_f'); ylabel('\mu');
    title(sprintf('(%c) %s, area=%.2f','a'+i-1,methodLabel(methods(i)), ...
        R.boundary.areaRatio.(char(methods(i))))); colorbar;
end
nexttile; vals=zeros(numel(methods),1);
for i=1:numel(methods), vals(i)=R.boundary.areaRatio.(char(methods(i))); end
bar(vals); set(gca,'XTickLabel',methodLabel(methods),'XTickLabelRotation',25); ylim([0 1]);
ylabel('Recoverable-area ratio'); title('(f) Area comparison'); grid on;
finishFigure(fig,8,cfg);
end

function plotFigure9(R,cfg)
methods=R.monteCarlo.methods;
fig=newFigure('Figure 9 - Paired Monte Carlo'); tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
fields={"lateralRMSE","yawRMSE","peakRho","minRealizedPDAR"};
titles={'(a) Lateral RMSE','(b) Yaw-rate RMSE','(c) Peak utilization','(d) Minimum realized PDAR'};
for p=1:4
    nexttile; A=R.monteCarlo.metrics.(fields{p});
    plotDistributions(A,methods); ylabel(strrep(fields{p},'_',' ')); title(titles{p}); grid on;
end
finishFigure(fig,9,cfg);
end

function plotFigure10(R,cfg)
A=R.ablation.metricMatrix;
fig=newFigure('Figure 10 - Ablation'); tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; normalized=A(:,1:5)./max(abs(A(:,1:5)),[],1); bar(normalized);
set(gca,'XTickLabel',R.ablation.methods); ylabel('Normalized metric'); title('(a) A0-A4 metrics'); grid on;
legend(R.ablation.metricNames(1:5),'Location','best','FontSize',7);
nexttile; bar(diff(A(:,1:5),1,1)); set(gca,'XTickLabel',{'A1-A0','A2-A1','A3-A2','A4-A3'});
ylabel('Metric increment'); title('(b) Incremental contributions'); grid on;
nexttile; scatter(A(:,5),A(:,4),50,1:5,'filled'); text(A(:,5),A(:,4),"  "+R.ablation.methods);
xlabel('Suspension energy (kJ)'); ylabel('Minimum PDAR'); title('(c) PDAR-energy trade-off'); grid on;
fallbacks = zeros(numel(R.ablation.methods),1);
for i=1:numel(R.ablation.methods)
    fallbacks(i) = R.ablation.outputs.(char(R.ablation.methods(i))).metrics.fallbackCount;
end
nexttile; bar([A(:,6),fallbacks]);
set(gca,'XTickLabel',R.ablation.methods); ylabel('Count / indicator'); title('(d) Failure and fallback'); grid on;
legend('Failure','Fallbacks'); finishFigure(fig,10,cfg);
end

function plotFigure11(R,cfg)
idx=find(R.delay.methods=="P_ASR",1);
fig=newFigure('Figure 11 - Health-input delay and bias'); tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
[B,D]=meshgrid(R.delay.bias,R.delay.delay*1000);
nexttile; surf(B,D,R.delay.failureProbability(:,:,idx)); xlabel('Health bias'); ylabel('Delay (ms)'); zlabel('Failure probability'); title('(a) Failure surface'); view(135,30); colorbar;
nexttile; surf(B,D,R.delay.minPDAR(:,:,idx)); xlabel('Health bias'); ylabel('Delay (ms)'); zlabel('Minimum PDAR'); title('(b) Reserve surface'); view(135,30); colorbar;
nexttile; surf(B,D,R.delay.peakRho(:,:,idx)); xlabel('Health bias'); ylabel('Delay (ms)'); zlabel('Peak \rho'); title('(c) Utilization surface'); view(135,30); colorbar;
nexttile; vals=squeeze(mean(R.delay.failureProbability,2)); bar(vals); set(gca,'XTickLabel',compose('%g ms',R.delay.delay*1000));
ylabel('Mean failure probability'); title('(d) Multi-method delay comparison'); legend(methodLabel(R.delay.methods),'Location','best'); grid on;
finishFigure(fig,11,cfg);
end

function plotFigure12(R,cfg)
idx=find(R.mismatch.methods=="P_ASR",1);
fig=newFigure('Figure 12 - Suspension mismatch'); tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
[T,F]=meshgrid(R.mismatch.tauScale,R.mismatch.forceScale);
nexttile; surf(T,F,R.mismatch.failureProbability(:,:,idx)); xlabel('\kappa_\tau'); ylabel('\kappa_F'); zlabel('Failure probability'); title('(a) Recoverable outcome'); view(135,30); colorbar;
nexttile; surf(T,F,R.mismatch.falseRecoverableRate(:,:,idx)); xlabel('\kappa_\tau'); ylabel('\kappa_F'); zlabel('False-recoverable rate'); title('(b) Gate mismatch'); view(135,30); colorbar;
nexttile; surf(T,F,R.mismatch.minPDAR(:,:,idx)); xlabel('\kappa_\tau'); ylabel('\kappa_F'); zlabel('Minimum PDAR'); title('(c) Reserve'); view(135,30); colorbar;
nexttile; surf(T,F,R.mismatch.energy(:,:,idx)); xlabel('\kappa_\tau'); ylabel('\kappa_F'); zlabel('Energy (kJ)'); title('(d) Suspension energy'); view(135,30); colorbar;
finishFigure(fig,12,cfg);
end

function boundary = boundary2D(theta,Fz,Fy,lambda,mu,delta,cfg)
boundary=zeros(2,numel(theta));
for i=1:numel(theta)
    g=[cfg.scale.Fref*cos(theta(i));cfg.scale.Mzref*sin(theta(i));0];
    gamma=directionalMargin(g,Fz,Fy,lambda,mu,delta,cfg,false);
    boundary(:,i)=gamma*g(1:2);
end
end

function compareTrace(S,methods,field,scale,yLabelText,titleText)
for m=methods
    H=S.(char(m)).history; plot(H.t,scale*H.(field),'LineWidth',1.0); hold on;
end
xlabel('Time (s)'); ylabel(yLabelText); title(titleText); grid on;
end

function plotDistributions(A,methods)
n=size(A,1); hold on;
for j=1:size(A,2)
    x=j+0.14*(rand(n,1)-0.5);
    scatter(x,A(:,j),16,'filled','MarkerFaceAlpha',0.45);
    med=median(A(:,j),'omitnan');
    plot([j-0.22,j+0.22],[med,med],'k-','LineWidth',1.5);
end
xlim([0.5,numel(methods)+0.5]); set(gca,'XTick',1:numel(methods),'XTickLabel',methodLabel(methods),'XTickLabelRotation',25);
end

function fig = newFigure(name)
fig=figure('Color','w','Name',name,'Units','centimeters','Position',[2,2,18,13]);
end

function finishFigure(fig,number,cfg)
set(findall(fig,'-property','FontName'),'FontName',cfg.output.fontName);
set(findall(fig,'-property','FontSize'),'FontSize',cfg.output.fontSize);
if cfg.output.saveFigures
    png=fullfile(cfg.output.directory,sprintf('Figure_%02d.png',number));
    figFile=fullfile(cfg.output.directory,sprintf('Figure_%02d.fig',number));
    exportgraphics(fig,png,'Resolution',cfg.output.figureDpi);
    savefig(fig,figFile);
end
end

function label = methodLabel(method)
label=strrep(string(method),'_','-');
end
