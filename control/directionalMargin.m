function [gamma,u,status] = directionalMargin(gDemand,Fz,Fy,lambda,mu,delta,cfg,allowSuspension)
%DIRECTIONALMARGIN Maximum normalized scaling on the generalized-demand ray.

if nargin < 8
    allowSuspension = true;
end

gTilde = cfg.scale.Sg*gDemand;
if norm(gTilde,2) < 1e-9
    gamma = cfg.control.gammaCap;
    u = zeros(8,1);
    status = 1;
    return;
end

c = [cos(delta);cos(delta);1;1];
b = cfg.vehicle.xw.*[sin(delta);sin(delta);0;0] - cfg.vehicle.yw.*c;
B = [c.',zeros(1,4);b.',zeros(1,4);zeros(1,4),cfg.vehicle.yw.'];

motorCap = lambda(:)*cfg.motor.Tmax/cfg.vehicle.Rw;
frictionCap = sqrt(max((mu(:).*Fz(:)).^2-Fy(:).^2,0));
FxCap = max(0,min(motorCap,frictionCap)*cfg.tyre.tightening);
lb = [-FxCap;-cfg.susp.Fmax*ones(4,1)];
ub = [ FxCap; cfg.susp.Fmax*ones(4,1)];
if ~allowSuspension
    lb(5:8) = 0;
    ub(5:8) = 0;
end

Aeq = [cfg.scale.Sg*B,-gTilde];
beq = zeros(3,1);
f = [zeros(8,1);-1];
lbLP = [lb;0];
ubLP = [ub;cfg.control.gammaCap];

status = -1;
z = [];
if exist('linprog','file') == 2
    try
        opts = optimoptions('linprog','Display','none','Algorithm','dual-simplex');
        [z,~,status] = linprog(f,[],[],Aeq,beq,lbLP,ubLP,opts);
    catch
        z = [];
    end
end

if isempty(z) || status <= 0
    [gamma,u,status] = rayFallback(cfg.scale.Sg*B,gTilde,lb,ub,cfg.control.gammaCap);
else
    u = z(1:8);
    gamma = max(0,z(9));
end
end

function [gamma,u,status] = rayFallback(Bs,g,lb,ub,gammaCap)
% Feasible lower bound when Optimization Toolbox is unavailable.
uRay = pinv(Bs)*g;
ratios = gammaCap;
for i = 1:numel(uRay)
    if uRay(i) > 1e-10
        ratios(end+1) = ub(i)/uRay(i); %#ok<AGROW>
    elseif uRay(i) < -1e-10
        ratios(end+1) = lb(i)/uRay(i); %#ok<AGROW>
    end
end
gamma = max(0,min(ratios));
u = gamma*uRay;
u = min(max(u,lb),ub);
status = 0;
end
