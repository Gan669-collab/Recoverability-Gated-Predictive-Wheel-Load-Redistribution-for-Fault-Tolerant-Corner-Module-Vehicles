function plotSingleCase(out,cfg)
%PLOTSINGLECASE Compact diagnostic plot for one simulation.

H = out.history;
figure('Color','w','Name',char(out.method));
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; plot(H.X,H.Y,'LineWidth',1.3); hold on; plot(H.X,H.yRef,'--');
xlabel('X (m)'); ylabel('Y (m)'); legend('Vehicle','Reference','Location','best'); grid on;
nexttile; plot(H.t,rad2deg(H.r),'LineWidth',1.2); hold on; plot(H.t,rad2deg(H.rRef),'--');
xlabel('Time (s)'); ylabel('Yaw rate (deg/s)'); grid on;
nexttile; plot(H.t,H.GammaRide,'LineWidth',1.2); hold on; plot(H.t,H.GammaStar,'LineWidth',1.2);
yline(1+cfg.control.deltaGamma,'k--'); xlabel('Time (s)'); ylabel('PDAR'); grid on;
legend('\Gamma^{ride}','\Gamma^*','Threshold','Location','best');
nexttile; plot(H.t,H.rho','LineWidth',1.0); xlabel('Time (s)'); ylabel('Tyre utilization');
yline(1,'k--'); grid on;
sgtitle(strrep(char(out.method),'_',' '));
end

