function ref = referenceInput(t, x, scn, cfg)
%REFERENCEINPUT Prescribed path/speed references and road disturbance.

vx = max(x.vx, 2);
switch scn.name
    case "DLC"
        A = 3.2;
        w = 0.18;
        s1 = 0.5*(1+tanh((t-1.4)/w));
        s2 = 0.5*(1+tanh((t-2.6)/w));
        s3 = 0.5*(1+tanh((t-3.8)/w));
        s4 = 0.5*(1+tanh((t-5.0)/w));
        yRef = A*(s1-s2-s3+s4);
        dyRef = A/(2*w)*(sech((t-1.4)/w)^2-sech((t-2.6)/w)^2 ...
            -sech((t-3.8)/w)^2+sech((t-5.0)/w)^2);
        psiRef = atan2(dyRef, vx);
        vRef = scn.speed0;
        deltaFF = 0;
    case "SWD"
        amp = deg2rad(5.5);
        if t < 1.0
            deltaFF = 0;
        elseif t < 2.2
            deltaFF = amp*sin(pi*(t-1.0)/1.2);
        elseif t < 2.8
            deltaFF = amp;
        elseif t < 4.0
            deltaFF = 0.5*amp*(1+cos(pi*(t-2.8)/1.2));
        else
            deltaFF = 0;
        end
        yRef = x.Y;
        psiRef = x.psi + deltaFF*0.15;
        vRef = scn.speed0;
    case "SPLIT"
        deltaFF = deg2rad(3.2)*sin(pi*max(t-0.8,0)/2.6) * double(t < 3.4);
        yRef = 1.0*0.5*(1+tanh((t-1.5)/0.35));
        psiRef = 0.08*0.5*(1+tanh((t-1.5)/0.35));
        vRef = max(5, scn.speed0 - 4.0*max(t-1.0,0));
    otherwise
        error('Unsupported scenario.');
end

deltaFB = cfg.control.Kpath*(yRef-x.Y) + cfg.control.Kheading*wrapToPiLocal(psiRef-x.psi);
ref.delta = min(max(deltaFF + deltaFB, -cfg.control.deltaMax), cfg.control.deltaMax);
ref.y = yRef;
ref.psi = psiRef;
ref.v = vRef;
ref.r = vx/cfg.vehicle.L*tan(ref.delta);

muLim = min([scn.muLeft,scn.muRight])*cfg.sim.g/max(vx,2);
ref.r = min(max(ref.r,-muLim),muLim);

phase = [0;0.8;1.6;2.4];
ref.road = scn.roadAmplitude * sin(2*pi*1.2*t + phase);
end

function a = wrapToPiLocal(a)
a = mod(a+pi,2*pi)-pi;
end
