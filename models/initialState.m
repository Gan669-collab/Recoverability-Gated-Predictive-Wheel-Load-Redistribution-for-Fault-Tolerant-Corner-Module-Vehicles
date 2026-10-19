function x = initialState(scn, cfg)
%INITIALSTATE Initialize Plant A states.

x.X = 0;
x.Y = 0;
x.psi = 0;
x.vx = scn.speed0;
x.vy = 0;
x.r = 0;
x.phi = 0;
x.phidot = 0;
x.theta = 0;
x.thetadot = 0;
x.zs = 0;
x.zsdot = 0;
x.zu = zeros(4,1);
x.zudot = zeros(4,1);
x.omega = scn.speed0/cfg.vehicle.Rw*ones(4,1);
x.Tact = zeros(4,1);
x.Faact = zeros(4,1);
x.Fx = zeros(4,1);
x.Fy = zeros(4,1);
x.Fz = cfg.vehicle.Fz0*scn.parameterScale.mass;
x.ax = 0;
x.ay = 0;
x.stroke = zeros(4,1);
end
