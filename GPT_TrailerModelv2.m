%% Trailer Model (Euler-angle version: ZYX, Newton–Euler)
clear; clc; close all;

%% ---------------------------- PARAMETERS ---------------------------------
g = 9.81;

% Body (sprung)
mB   = 1200;
IB   = diag([450, 2200, 2500]);

% Geometry (body frame: x fwd, y left, z up)
trackF = 1.8;  trackR = 1.8;
axleF_x = +1.2;            % front axle x from CG
axleR_x = -1.2;            % rear  axle x from CG
seat_zB = -0.40;           % seat/bushing z on body

% Body hardpoints (front/rear, L/R)
r_F_L_B = [axleF_x; +trackF/2; seat_zB];
r_F_R_B = [axleF_x; -trackF/2; seat_zB];
r_R_L_B = [axleR_x; +trackR/2; seat_zB];
r_R_R_B = [axleR_x; -trackR/2; seat_zB];

% Axle local positions of wheel centers (in axle frames)
r_LF_A  = [0; +trackF/2; 0];
r_RF_A  = [0; -trackF/2; 0];
r_LR_A  = [0; +trackR/2; 0];
r_RR_A  = [0; -trackR/2; 0];

% Unsprung masses (axle bodies)
mAF = 100;  IAF = diag([20, 40, 40]);
mAR = 110;  IAR = diag([22, 44, 44]);

% Seat bushings (3DOF translational per corner)
Kseat = diag([2e5, 5e5, 6e4]);    % N/m   (x,y,z)
Cseat = diag([2e3, 8e3, 1.2e4]);  % N*s/m

% Wheels / tires
Rw   = 0.35;                % radius
Jw   = 1.8;                 % wheel inertia (rotational)
Tloss_c = 0.01;             % small viscous spin loss [N*m*s]

% Longitudinal Magic Formula (simple)
Bx = 8;    Cx = 1.5;    Dx = 0.8*2500;    Ex = 0.0;   % Ex=0 if unknown
Bx_cap = 6;                                % cap to avoid steep stiffness

% Lateral cornering
Cy = 7.5e4;                % N/rad
mu = 1.0;                  % friction coeff
epsv = 0.2;                % small regularizer for |vx|
vblend = 0.5;              % [m/s] low-speed blending range for slip/alpha

% Rolling resistance (force level)
Crr = 0.015;

% Tire vertical (SOFT penalty; faster numerics)
ktc = 1.2e5;
ctc = 4.0e3;

% Aero on body
rho=1.225; Af=2.2; Cd=0.8; CyawM=400; Wind_E=[0;0;0];

% Steering commands (per axle, radians)
deltaF_fun = @(t) 0.0;
deltaR_fun = @(t) 0.0;

% Brake torques per wheel (set functions; zero = free roll)
Tb_LF_fun = @(t) 0.0;
Tb_RF_fun = @(t) 0.0;
Tb_LR_fun = @(t) 0.0;
Tb_RR_fun = @(t) 0.0;

% ---- Road profiles (time-based scenarios, per wheel) ----
% Available names: "sin", "sin_antiphase", "speedbump", "washboard",
% "washboard_phase", "step_up", "step_down", "trench",
% "ramp_up", "ramp_down", "random"
roadScenario = "sin";   % pick any from the list below
roadA  = 0.015*0;         % amplitude [m], 0 for no profile
roadF  = 1.0;           % base frequency [Hz]

% Build per-wheel time functions z(t) for LF/RF/LR/RR
[ zRoadLF_fun, zRoadRF_fun, zRoadLR_fun, zRoadRR_fun ] = ...
    road_profile_handles(roadScenario, roadA, roadF);

% Wheel sync (only near zero speed)
tau_sync   = 0.4;        % gentler sync
v_sync_on  = 0.5;        % engage sync when |vx_w| < this

% Hitch inputs (in BODY coordinates)
Fxh_fun = @(t) 700 + 0.0.*t;  % longitudinal pull/push
Fyh_fun = @(t) 700 + 0.0.*t;         % lateral
Fzh_fun = @(t) 0.0.*t;         % vertical
Mxh_fun = @(t) 0.0.*t;  % roll moment
Myh_fun = @(t) 0.0.*t;         % pitch moment
Mzh_fun = @(t) 0.0.*t;         % yaw moment

P.Fxh_fun = Fxh_fun; P.Fyh_fun = Fyh_fun; P.Fzh_fun = Fzh_fun;
P.Mxh_fun = Mxh_fun; P.Myh_fun = Myh_fun; P.Mzh_fun = Mzh_fun;
P.rB_H    = [2; 0; 0.3];    % hitch location in body frame

%% --------------------------- INITIAL CONDITIONS --------------------------
% Euler angles (ZYX): [phi; theta; psi] = [roll; pitch; yaw]
phiB0   = 0; thetaB0 = 0; psiB0 = 0;
phiAF0  = 0; thetaAF0= 0; psiAF0= 0;
phiAR0  = 0; thetaAR0= 0; psiAR0= 0;

z_pre = 0.01;                         % initial tire compression
zW_F  = zRoadLF_fun(0) + Rw - z_pre;
zW_R  = zRoadLR_fun(0) + Rw - z_pre;

rAF0 = [axleF_x; 0; zW_F];
rAR0 = [axleR_x; 0; zW_R];
rB0  = [0; 0; zW_F - r_F_L_B(3)];

vB0  = [20;0;0];  wB0  = [0;0;0];
vAF0 = vB0;       wAF0 = [0;0;0];
vAR0 = vB0;       wAR0 = [0;0;0];

omega0 = vB0(1)/Rw;

% State layout:
% Body:  rB(3), phiB,thetaB,psiB (3), vB(3), wB(3) = 12
% AxleF: rAF(3),phiAF,thetaAF,psiAF(3),vAF(3),wAF(3) = 12
% AxleR: rAR(3),phiAR,thetaAR,psiAR(3),vAR(3),wAR(3) = 12
% Wheels: omegaLF, omegaRF, omegaLR, omegaRR = 4
x0 = [ rB0;
       phiB0; thetaB0; psiB0;
       vB0; wB0; ...
       rAF0;
       phiAF0; thetaAF0; psiAF0;
       vAF0; wAF0; ...
       rAR0;
       phiAR0; thetaAR0; psiAR0;
       vAR0; wAR0; ...
       omega0; omega0; omega0; omega0 ];

%% ------------------------------ PACK CONST -------------------------------
P.g=g; P.mB=mB; P.IB=IB;
P.r_F_L_B=r_F_L_B; P.r_F_R_B=r_F_R_B; P.r_R_L_B=r_R_L_B; P.r_R_R_B=r_R_R_B;
P.r_LF_A=r_LF_A; P.r_RF_A=r_RF_A; P.r_LR_A=r_LR_A; P.r_RR_A=r_RR_A;
P.mAF=mAF; P.IAF=IAF; P.mAR=mAR; P.IAR=IAR; P.Kseat=Kseat; P.Cseat=Cseat;

P.Rw=Rw; P.Jw=Jw; P.Bx=Bx; P.Cx=Cx; P.Dx=Dx; P.Ex=Ex; P.Bx_cap=Bx_cap;
P.Cy=Cy; P.mu=mu; P.epsv=epsv; P.Crr=Crr; P.vblend=vblend;
P.ktc=ktc; P.ctc=ctc; P.Tloss_c=Tloss_c;

P.rho=rho; P.Af=Af; P.Cd=Cd; P.CyawM=CyawM; P.Wind_E=Wind_E;

P.deltaF_fun=deltaF_fun; P.deltaR_fun=deltaR_fun;
P.zRoadLF_fun=zRoadLF_fun; P.zRoadRF_fun=zRoadRF_fun;
P.zRoadLR_fun=zRoadLR_fun; P.zRoadRR_fun=zRoadRR_fun;

P.Tb_LF_fun=Tb_LF_fun; P.Tb_RF_fun=Tb_RF_fun;
P.Tb_LR_fun=Tb_LR_fun; P.Tb_RR_fun=Tb_RR_fun;

P.tau_sync=tau_sync; P.v_sync_on=v_sync_on;

%% -------------------------------- SOLVER ---------------------------------
tEnd=30;
S = jpattern_trailer();
opts=odeset('RelTol',2e-4,'AbsTol',2e-6,'MaxStep',6e-2,'InitialStep',2e-3, ...
            'JPattern',S,'Stats','on', ...
            'Events',@(t,x) stop_when_speed_zero(t,x,P));
[t,X]=ode15s(@(t,x) ode_trailer(t,x,P), [0 tEnd], x0, opts);

%% -------------------------------- PLOTS ----------------------------------
plot_demo_style(t, X, P);

% Generate hitch timeseries
Fxh_ts = timeseries(P.Fxh_fun(t), t, 'Name','Fxh');
Fyh_ts = timeseries(P.Fyh_fun(t), t, 'Name','Fyh');
Fzh_ts = timeseries(P.Fzh_fun(t), t, 'Name','Fzh');
Mxh_ts = timeseries(P.Mxh_fun(t), t, 'Name','Mxh');
Myh_ts = timeseries(P.Myh_fun(t), t, 'Name','Myh');
Mzh_ts = timeseries(P.Mzh_fun(t), t, 'Name','Mzh');

zLF_ts = timeseries(zRoadLF_fun(t), t, 'Name','zLF');
zRF_ts = timeseries(zRoadRF_fun(t), t, 'Name','zRF');
zLR_ts = timeseries(zRoadLR_fun(t), t, 'Name','zLR');
zRR_ts = timeseries(zRoadRR_fun(t), t, 'Name','zRR');

deltaF_ts = timeseries(deltaF_fun(t), t, 'Name','deltaF');
deltaR_ts = timeseries(deltaR_fun(t), t, 'Name','deltaR');

%% =============================== ODE CORE ================================
function xdot = ode_trailer(t,x,P)
    i=1;
    % ---- Body ----
    rB = x(i:i+2); i=i+3;
    phiB   = x(i); thetaB = x(i+1); psiB = x(i+2); i = i+3;
    vB = x(i:i+2); i=i+3;
    wB = x(i:i+2); i=i+3;
    
    % ---- Front Axle ----
    rAF = x(i:i+2); i=i+3;
    phiAF   = x(i); thetaAF = x(i+1); psiAF = x(i+2); i = i+3;
    vAF = x(i:i+2); i=i+3;
    wAF = x(i:i+2); i=i+3;
    
    % ---- Rear Axle ----
    rAR = x(i:i+2); i=i+3;
    phiAR   = x(i); thetaAR = x(i+1); psiAR = x(i+2); i = i+3;
    vAR = x(i:i+2); i=i+3;
    wAR = x(i:i+2); i=i+3;
    
    % ---- Wheel spins ----
    omegaLF = x(i); omegaRF = x(i+1); omegaLR = x(i+2); omegaRR = x(i+3);
    
    % Rotation matrices from Euler ZYX angles
    RB   = eulZYX_to_R(phiB,thetaB,psiB);
    RAF  = eulZYX_to_R(phiAF,thetaAF,psiAF);
    RAR  = eulZYX_to_R(phiAR,thetaAR,psiAR);
    
    % two seat bushings (front L/R + rear L/R)
    [Fb_B, Mb_B, F_F_A, M_F_A, F_R_A, M_R_A] = ...
        two_point_bushings(rB,vB,wB,RB, rAF,vAF,wAF,RAF, rAR,vAR,wAR,RAR, P);
    
    % steering angles (per axle)
    dF = P.deltaF_fun(t); dR = P.deltaR_fun(t);
    
    % wheel kinematics (centers in world and velocities)
    [pLF_E,vLF_E] = wheel_kin(rAF,RAF,vAF,wAF,P.r_LF_A);
    [pRF_E,vRF_E] = wheel_kin(rAF,RAF,vAF,wAF,P.r_RF_A);
    [pLR_E,vLR_E] = wheel_kin(rAR,RAR,vAR,wAR,P.r_LR_A);
    [pRR_E,vRR_E] = wheel_kin(rAR,RAR,vAR,wAR,P.r_RR_A);
    
    % road heights
    zLF = P.zRoadLF_fun(t); zRF = P.zRoadRF_fun(t); 
    zLR = P.zRoadLR_fun(t); zRR = P.zRoadRR_fun(t);
    
    % vertical contact (smoothed)
    FzLF = tire_normal_soft(pLF_E(3), vLF_E(3), zLF, P.Rw, P.ktc, P.ctc);
    FzRF = tire_normal_soft(pRF_E(3), vRF_E(3), zRF, P.Rw, P.ktc, P.ctc);
    FzLR = tire_normal_soft(pLR_E(3), vLR_E(3), zLR, P.Rw, P.ktc, P.ctc);
    FzRR = tire_normal_soft(pRR_E(3), vRR_E(3), zRR, P.Rw, P.ktc, P.ctc);
    
    % tire forces (wheel frame -> axle frame)
    [FxLF_A,FyLF_A,FxLF_w,vxLF_w] = tire_forces_MF(vLF_E,RAF,dF,omegaLF,FzLF,P);
    [FxRF_A,FyRF_A,FxRF_w,vxRF_w] = tire_forces_MF(vRF_E,RAF,dF,omegaRF,FzRF,P);
    [FxLR_A,FyLR_A,FxLR_w,vxLR_w] = tire_forces_MF(vLR_E,RAR,dR,omegaLR,FzLR,P);
    [FxRR_A,FyRR_A,FxRR_w,vxRR_w] = tire_forces_MF(vRR_E,RAR,dR,omegaRR,FzRR,P);
    
    % friction ellipse (smooth scaling)
    [FxLF_A,FyLF_A,sLF]=friction_ellipse_smooth(FxLF_A,FyLF_A,FzLF,P.mu); %#ok<NASGU>
    [FxRF_A,FyRF_A,sRF]=friction_ellipse_smooth(FxRF_A,FyRF_A,FzRF,P.mu); %#ok<NASGU>
    [FxLR_A,FyLR_A,sLR]=friction_ellipse_smooth(FxLR_A,FyLR_A,FzLR,P.mu); %#ok<NASGU>
    [FxRR_A,FyRR_A,sRR]=friction_ellipse_smooth(FxRR_A,FyRR_A,FzRR,P.mu); %#ok<NASGU>
    
    % Use the ACTUAL (limited) contact forces to drive wheel spin.
    % Rotate axle->wheel to get the x (longitudinal) component in wheel frame:
    cF = cos(dF); sF = sin(dF); cR = cos(dR); sR = sin(dR);
    
    FxLF_w_torque =  cF*FxLF_A + sF*FyLF_A; 
    FxRF_w_torque =  cF*FxRF_A + sF*FyRF_A;
    FxLR_w_torque =  cR*FxLR_A + sR*FyLR_A; 
    FxRR_w_torque =  cR*FxRR_A + sR*FyRR_A;
    
    % add verticals (in axle frames) and sum axle resultants
    FzLF_A = RAF'*[0;0;FzLF]; FzRF_A = RAF'*[0;0;FzRF];
    FzLR_A = RAR'*[0;0;FzLR]; FzRR_A = RAR'*[0;0;FzRR];
    FwLF_A = [FxLF_A;FyLF_A;0] + FzLF_A;
    FwRF_A = [FxRF_A;FyRF_A;0] + FzRF_A;
    FwLR_A = [FxLR_A;FyLR_A;0] + FzLR_A;
    FwRR_A = [FxRR_A;FyRR_A;0] + FzRR_A;
    
    F_axF_A = F_F_A + FwLF_A + FwRF_A;
    M_axF_A = M_F_A + cross3(P.r_LF_A,FwLF_A) + cross3(P.r_RF_A,FwRF_A);
    F_axR_A = F_R_A + FwLR_A + FwRR_A;
    M_axR_A = M_R_A + cross3(P.r_LR_A,FwLR_A) + cross3(P.r_RR_A,FwRR_A);
    
    % body forces/moments at CG
    Fg_B    = RB'*[0;0;-P.mB*P.g];
    Faero_B = aero_body(vB,RB,P.Wind_E,P.rho,P.Af,P.Cd);
    Maero_B = [0;0;-P.CyawM*vB(2)];
    F_body_B = Fb_B + Fg_B + Faero_B;
    M_body_B = Mb_B + Maero_B;
    
    % Hitch force and moment (in body frame)
    Fh_B = [ P.Fxh_fun(t);
             P.Fyh_fun(t);
             P.Fzh_fun(t) ];
    
    Mh_B = [ P.Mxh_fun(t);
             P.Myh_fun(t);
             P.Mzh_fun(t) ];
    
    % Transform hitch moment to act at CG (rB_H is from CG to hitch, in body frame)
    M_hitch_total = Mh_B + cross3(P.rB_H, Fh_B);
    
    % Add hitch loads to body totals
    F_body_B = F_body_B + Fh_B;
    M_body_B = M_body_B + M_hitch_total;
    
    % ---- rigid-body dynamics: Body ----
    vBd = (F_body_B - cross3(wB,P.mB*vB))/P.mB;
    wBd = P.IB \ ( M_body_B - cross3(wB,P.IB*wB) );
    rBd = RB * vB;
    eBdot = eulRatesZYX_from_bodyRates(phiB,thetaB,wB);  % [phidot; thetadot; psidot]
    
    % ---- axle dynamics: Front ----
    Fg_AF_A = RAF'*[0;0;-P.mAF*P.g];
    vAFd = (F_axF_A + Fg_AF_A - cross3(wAF,P.mAF*vAF))/P.mAF;
    wAFd = P.IAF \ (M_axF_A - cross3(wAF,P.IAF*wAF));
    rAFd = RAF * vAF;
    eAFdot = eulRatesZYX_from_bodyRates(phiAF,thetaAF,wAF);
    
    % ---- axle dynamics: Rear ----
    Fg_AR_A = RAR'*[0;0;-P.mAR*P.g];
    vARd = (F_axR_A + Fg_AR_A - cross3(wAR,P.mAR*vAR))/P.mAR;
    wARd = P.IAR \ (M_axR_A - cross3(wAR,P.IAR*wAR));
    rARd = RAR * vAR;
    eARdot = eulRatesZYX_from_bodyRates(phiAR,thetaAR,wAR);
    
    % --- wheel-spin ODEs ---
    TbLF = P.Tb_LF_fun(t); TbRF = P.Tb_RF_fun(t);
    TbLR = P.Tb_LR_fun(t); TbRR = P.Tb_RR_fun(t);
    
    omegaLFd = ( FxLF_w_torque*P.Rw - TbLF - P.Tloss_c*omegaLF ) / P.Jw;
    omegaRFd = ( FxRF_w_torque*P.Rw - TbRF - P.Tloss_c*omegaRF ) / P.Jw;
    omegaLRd = ( FxLR_w_torque*P.Rw - TbLR - P.Tloss_c*omegaLR ) / P.Jw;
    omegaRRd = ( FxRR_w_torque*P.Rw - TbRR - P.Tloss_c*omegaRR ) / P.Jw;
    
    % keep the same gentle near-zero sync using vx*_w from tire_forces_MF:
    gainL  = 0.5*(P.v_sync_on / sqrt(vxLF_w*vxLF_w + P.v_sync_on^2));
    gainR  = 0.5*(P.v_sync_on / sqrt(vxRF_w*vxRF_w + P.v_sync_on^2));
    gainLR = 0.5*(P.v_sync_on / sqrt(vxLR_w*vxLR_w + P.v_sync_on^2));
    gainRR = 0.5*(P.v_sync_on / sqrt(vxRR_w*vxRR_w + P.v_sync_on^2));
    
    omegaLFd = omegaLFd + gainL *((vxLF_w/P.Rw) - omegaLF)/P.tau_sync;
    omegaRFd = omegaRFd + gainR *((vxRF_w/P.Rw) - omegaRF)/P.tau_sync;
    omegaLRd = omegaLRd + gainLR*((vxLR_w/P.Rw) - omegaLR)/P.tau_sync;
    omegaRRd = omegaRRd + gainRR*((vxRR_w/P.Rw) - omegaRR)/P.tau_sync;
    
    % assemble derivative
    xdot = [ rBd; eBdot; vBd; wBd; ...
             rAFd; eAFdot; vAFd; wAFd; ...
             rARd; eARdot; vARd; wARd; ...
             omegaLFd; omegaRFd; omegaLRd; omegaRRd ];
end

%% ============================ EVENT: STOP ================================
function [value,isterminal,direction] = stop_when_speed_zero(t,x,P)
    persistent tlast; if isempty(tlast), tlast = -Inf; end
    if t < 0.5 || (t - tlast) < 0.02
        value = 1; isterminal=1; direction=-1; return
    end
    tlast = t;

    % State layout for body:
    % rB(1:3), phiB(4), thetaB(5), psiB(6), vB(7:9), wB(10:12)
    vB = x(7:9);
    phiB   = x(4);
    thetaB = x(5);
    psiB   = x(6);
    RB = eulZYX_to_R(phiB,thetaB,psiB);
    vE = RB*vB;
    value = norm(vE) - 0.12;
    isterminal = 1; direction  = -1;
end

%% ============================ TIRE / FORCES ==============================
function [Fx_A,Fy_A,Fx_w,vx_w] = tire_forces_MF(vWheel_E,RA,delta,omega,Fz,P)
    % Put wheel-center velocity into axle frame
    vA = RA' * vWheel_E;
    
    % Rotate axle -> wheel frame by steer delta
    c=cos(delta); s=sin(delta);
    vx_w =  c*vA(1) + s*vA(2);
    vy_w = -s*vA(1) + c*vA(2);
    
    % Smooth low-speed blending
    vx_abs = sqrt(vx_w*vx_w + P.epsv^2);
    b  = vx_abs/(vx_abs + P.vblend);     % 0→1 as |vx| rises
    
    % UNIFIED longitudinal slip (always opposes mismatch), clamped [-1,1]
    lam = (vx_w - P.Rw*omega) / vx_abs;
    lam = clip(lam,-1,1);
    lam = b*lam;                        % blend to 0 at low |vx|
    
    % Lateral sideslip (simple) with blending
    alpha = b*atan2(vy_w, vx_abs);
    
    % Magic Formula (with stiffness cap)
    Bx_eff = min(P.Bx, P.Bx_cap);
    Fx_star = P.Dx * sin( P.Cx * atan( Bx_eff*lam - P.Ex*( Bx_eff*lam - atan(Bx_eff*lam) ) ) );
    
    % Lateral linear
    Fy_star = -P.Cy * alpha;
    
    % Rolling resistance with smooth sign
    sig = vx_w / sqrt(vx_w*vx_w + (0.2*P.vblend)^2);
    Fx_star = Fx_star - P.Crr * Fz * sig;
    
    % Transform back to axle frame
    Fx_A =  c*Fx_star - s*Fy_star;
    Fy_A =  s*Fx_star + c*Fy_star;
    
    % Return wheel-frame longitudinal for wheel ODE (used only for sync)
    Fx_w = Fx_star;
end

function [Fx_A,Fy_A,s]=friction_ellipse_smooth(Fx_A,Fy_A,Fz,mu)
    Flim = max(mu*Fz,1e-3);
    mag  = hypot(Fx_A,Fy_A) + 1e-12;
    s    = min(1, Flim./mag);
    Fx_A = Fx_A.*s;  Fy_A = Fy_A.*s;
end

function Fz = tire_normal_soft(zW, zWdot, zRoad, Rw, ktc, ctc)
    comp = (zRoad + Rw) - zW;
    hc   = 0.5*(comp + sqrt(comp*comp + 1e-6));          % smooth ReLU
    gate = 0.5*(1 + comp./sqrt(comp*comp + 1e-6));       % ~1 in contact
    Fz   = ktc*hc + ctc*gate*max(0,-zWdot);              % no pull when separating
end

%% ========================= BUSHINGS / KINEMATICS =========================
function [Fb_B, Mb_B, F_F_A, M_F_A, F_R_A, M_R_A] = ...
    two_point_bushings(rB,vB,wB,RB, rAF,vAF,wAF,RAF, rAR,vAR,wAR,RAR, P)

    [Fb_B, Mb_B]   = deal([0;0;0],[0;0;0]);
    [F_F_A, M_F_A] = deal([0;0;0],[0;0;0]);
    [F_R_A, M_R_A] = deal([0;0;0],[0;0;0]);
    
    function [Fb_B_i, Mb_B_i, F_A_i, M_A_i] = seat(rB,rSeatB_B,RB,vB,wB, ...
                                                   rA,rSeatA_A,RA,vA,wA, K,C)
        pB_E = rB + RB*rSeatB_B; 
        vB_E = RB*(vB + cross3(wB,rSeatB_B));
        pA_E = rA + RA*rSeatA_A; 
        vA_E = RA*(vA + cross3(wA,rSeatA_A));
        d_B  = RB'*(pA_E - pB_E);
        vd_B = RB'*(vA_E - vB_E);
        F_B  = K*d_B + C*vd_B;
        M_B  = cross3(rSeatB_B, F_B);
        Fb_B_i = F_B;  Mb_B_i = M_B;
        F_A_i = RA' * ( -RB*F_B ); 
        M_A_i = cross3(rSeatA_A, F_A_i);
    end
    
    [Fb1,Mb1,FA1,MA1] = seat(rB,P.r_F_L_B,RB, vB,wB, rAF,P.r_LF_A,RAF, vAF,wAF, P.Kseat,P.Cseat);
    [Fb2,Mb2,FA2,MA2] = seat(rB,P.r_F_R_B,RB, vB,wB, rAF,P.r_RF_A,RAF, vAF,wAF, P.Kseat,P.Cseat);
    Fb_B = Fb_B + Fb1 + Fb2;  Mb_B = Mb_B + Mb1 + Mb2;
    F_F_A= F_F_A+ FA1 + FA2;  M_F_A= M_F_A+ MA1 + MA2;
    
    [Fb3,Mb3,FA3,MA3] = seat(rB,P.r_R_L_B,RB, vB,wB, rAR,P.r_LR_A,RAR, vAR,wAR, P.Kseat,P.Cseat);
    [Fb4,Mb4,FA4,MA4] = seat(rB,P.r_R_R_B,RB, vB,wB, rAR,P.r_RR_A,RAR, vAR,wAR, P.Kseat,P.Cseat);
    Fb_B = Fb_B + Fb3 + Fb4;  Mb_B = Mb_B + Mb3 + Mb4;
    F_R_A= F_R_A+ FA3 + FA4;  M_R_A= M_R_A+ MA3 + MA4;
end

function [pE,vE] = wheel_kin(rA,RA,vA,wA,r_w_A)
    pE = rA + RA*r_w_A;
    vE = RA*(vA + cross3(wA, r_w_A));
end

%% ================================ HELPERS ================================
function C = cross3(a,b)
    C=[a(2)*b(3)-a(3)*b(2);
       a(3)*b(1)-a(1)*b(3);
       a(1)*b(2)-a(2)*b(1)];
end

function R = eulZYX_to_R(phi,theta,psi)
    cphi=cos(phi); sphi=sin(phi);
    cth =cos(theta); sth =sin(theta);
    cps =cos(psi); sps =sin(psi);
    R = [ cps*cth,               cps*sth*sphi - sps*cphi,   cps*sth*cphi + sps*sphi;
          sps*cth,               sps*sth*sphi + cps*cphi,   sps*sth*cphi - cps*sphi;
          -sth,                  cth*sphi,                  cth*cphi ];
end

function euldot = eulRatesZYX_from_bodyRates(phi,theta,omega)
    % omega = [p;q;r] in body frame; euldot = [phidot; thetadot; psidot]
    p = omega(1); q = omega(2); r = omega(3);
    cphi=cos(phi); sphi=sin(phi);
    cth =cos(theta); sth =sin(theta);
    epsc = 1e-8; cth = sign(cth)*max(abs(cth),epsc);
    tanth = sth/cth;
    euldot = [ 1, sphi*tanth,  cphi*tanth;
               0, cphi,        -sphi;
               0, sphi/cth,    cphi/cth ] * [p;q;r];
end

function Faero_B = aero_body(vB_B, RB_E, Wind_E, rho, Af, Cd)
    Vrel_B = vB_B - (RB_E'*Wind_E); 
    Va = norm(Vrel_B);
    Faero_B = -0.5*rho*Af*Cd*Va*(Va>0)*Vrel_B;
end

function [zLF,zRF,zLR,zRR] = road_profile_handles(name, A, f)
% Return four function handles z(t) for LF/RF/LR/RR wheels,
% implementing the same scenarios as your LTI demo.
% name: string or char (see cases below)
% A: amplitude [m] (default 0.015)
% f: base frequency [Hz] (default 4)

if nargin<2 || isempty(A), A = 0.015; end
if nargin<3 || isempty(f), f = 4.0;   end
name = string(lower(name));

% default: flat
z0 = @(t) 0.*t;

switch name
    case "sin"
        zL = @(t) A*sin(2*pi*f*t);
        zR = zL;

    case "sin_antiphase"
        zL = @(t) A*sin(2*pi*f*t);
        zR = @(t) -zL(t);

    case "speedbump"   % single smooth half-cosine bump vs time
        t0 = 1.5; w = 0.6;
        zL = @(t) A*( (t>=t0)&(t<=t0+w) ) .* (1 - cos(pi*(t-t0)/w));
        zR = zL;

    case "washboard"
        zL = @(t) A*0.7*(0.5 + 0.5*sin(2*pi*f*t));
        zR = zL;

    case "washboard_phase"
        zL = @(t) A*0.7*(0.5 + 0.5*sin(2*pi*f*t));
        zR = @(t) A*0.7*(0.5 + 0.5*sin(2*pi*f*t + pi/2));

    case "step_up"
        t0 = 1.5;
        zL = @(t) A*(t>=t0);
        zR = zL;

    case "step_down"
        t0 = 1.5;
        zL = @(t) -A*(t>=t0);
        zR = zL;

    case "trench"
        t1 = 1.2; t2 = 1.8;
        zL = @(t) -A.*((t>=t1)&(t<=t2));
        zR = zL;

    case "ramp_up"
        t0=1; t1=3;
        s = @(t) max(0,min(1,(t-t0)/(t1-t0)));
        zL = @(t) A*s(t);
        zR = zL;

    case "ramp_down"
        t0=1; t1=3;
        s = @(t) max(0,min(1,(t-t0)/(t1-t0)));
        zL = @(t) A*(1 - s(t));
        zR = zL;

    case "random"   % band-limited noise, deterministic seed
        rng(1);
        % Pre-generate on a coarse time grid, then interpolate inside ODE
        Tgrid = linspace(0, 60, 6001);    % 10 ms grid up to 60 s
        sigL = A*0.5*randn(size(Tgrid));
        sigR = A*0.5*randn(size(Tgrid));
        sigL = lowpass_magonly(sigL, Tgrid, 6);   % ~6 Hz cutoff
        sigR = lowpass_magonly(sigR, Tgrid, 6);
        zL = @(t) interp1(Tgrid, sigL, t, 'pchip', 'extrap');
        zR = @(t) interp1(Tgrid, sigR, t, 'pchip', 'extrap');

    otherwise
        warning('Unknown road scenario "%s". Using flat road.', name);
        zL = z0; zR = z0;
end

% Map to the four wheels (you can customize split profiles if desired)
zLF = zL;  zRF = zR;   % fronts
zLR = zL;  zRR = zR;   % rears (same as fronts by default)

end

function y = lowpass_magonly(x, t, fc)
% zero-phase magnitude-only lowpass via FFT (same idea as your demo)
X = fft(x); N = numel(x); dt = mean(diff(t));
f = (0:N-1)/(N*dt);
H = 1./sqrt(1+(f/fc).^8);
if mod(N,2)==0
    H([1, N/2+2:end]) = H([1, N/2:-1:2]);
else
    H([1, (N+3)/2:end]) = H([1, (N+1)/2:-1:2]);
end
Y = X .* H;
y = real(ifft(Y));
end

function v = clip(v,lo,hi), v=min(max(v,lo),hi); end

%% ---------------------------- JACOBIAN SPARSITY --------------------------
function S = jpattern_trailer()
% state layout: Body(12) | AxleF(12) | AxleR(12) | Wheels(4) = 40
nB=12; nA=12; nW=4; N=3*nA+nW;
S = spalloc(N,N, 600);
iB  = 1:nB;
iAF = nB+(1:nA);
iAR = nB+nA+(1:nA);
iW  = N-nW+1:N;

% self-couplings
S(iB,iB)=1;   S(iAF,iAF)=1;   S(iAR,iAR)=1;   S(iW,iW)=1;

% body–axle couplings (bushings)
S(iB,iAF)=1; S(iB,iAR)=1; S(iAF,iB)=1; S(iAR,iB)=1;

% axle–wheel couplings (tire/wheel ODE)
S(iAF,iW)=1; S(iAR,iW)=1; S(iW,iAF)=1; S(iW,iAR)=1;

% mild cross-couplings between axles via body
S(iAF,iAR)=1; S(iAR,iAF)=1;
end

%% ================================ PLOTS ==================================
function plot_demo_style(t, X, P)
% Five figures:
% 1) Hitch forces/moments
% 2) Road displacement & rate per wheel
% 3) Trailer body states (x,y,z, roll,pitch,yaw, u,v,w, p,q,r)
% 4) Front axle states (x,y,z, roll,pitch,yaw)
% 5) Rear axle states  (x,y,z, roll,pitch,yaw)

% ------------ unpack state ------------
% state layout: Body(12) | AxleF(12) | AxleR(12) | Wheels(4) = 40
i=1;
rB   = X(:,i:i+2); i=i+3;
phiB = X(:,i); thetaB = X(:,i+1); psiB = X(:,i+2); i = i+3;
vB   = X(:,i:i+2); i=i+3;
wB   = X(:,i:i+2); i=i+3;

rAF   = X(:,i:i+2); i=i+3;
phiAF = X(:,i); thetaAF = X(:,i+1); psiAF = X(:,i+2); i = i+3;
vAF   = X(:,i:i+2); i=i+3;
wAF   = X(:,i:i+2); i=i+3;

rAR   = X(:,i:i+2); i=i+3;
phiAR = X(:,i); thetaAR = X(:,i+1); psiAR = X(:,i+2); i = i+3;
vAR   = X(:,i:i+2); i=i+3;
wAR   = X(:,i:i+2); i=i+3; %#ok<NASGU,ASGLU>

% ------------ inputs (functions of time) ------------
Fxh = fun_or_zero(P,'Fxh_fun',t); Fyh = fun_or_zero(P,'Fyh_fun',t);
Fzh = fun_or_zero(P,'Fzh_fun',t); Mxh = fun_or_zero(P,'Mxh_fun',t);
Myh = fun_or_zero(P,'Myh_fun',t); Mzh = fun_or_zero(P,'Mzh_fun',t);

zLF = fun_or_zero(P,'zRoadLF_fun',t); zRF = fun_or_zero(P,'zRoadRF_fun',t);
zLR = fun_or_zero(P,'zRoadLR_fun',t); zRR = fun_or_zero(P,'zRoadRR_fun',t);

% numerical time derivatives for road rates
dt = max(eps, mean(diff(t)));
dzLF = gradient(zLF, dt); dzRF = gradient(zRF, dt);
dzLR = gradient(zLR, dt); dzRR = gradient(zRR, dt);

% ------------ Euler angles in deg ------------
deg   = 180/pi;
phiBd   = phiB*deg;  thetaBd = thetaB*deg;  psiBd = psiB*deg;
phiAFd  = phiAF*deg; thetaAFd= thetaAF*deg; psiAFd= psiAF*deg;
phiARd  = phiAR*deg; thetaARd= thetaAR*deg; psiARd= psiAR*deg;

% =================== Figure 1: Hitch forces/moments ======================
figure('Name','Hitch Inputs','NumberTitle','off','Color','w');
tl1 = tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
sgtitle(tl1,'Hitch Forces and Moments');
labs1 = {'F_xh [N]','F_yh [N]','F_zh [N]','M_xh [N·m]','M_yh [N·m]','M_zh [N·m]'};
sig1  = {Fxh,Fyh,Fzh,Mxh,Myh,Mzh};
for k=1:6
    nexttile; plot(t, sig1{k}, 'LineWidth', 1.1); grid on;
    xlabel('Time [s]'); title(labs1{k});
end

% = Figure 2: Road displacement and rate per wheel (LF/RF/LR/RR) ==========
figure('Name','Road Profiles','NumberTitle','off','Color','w');
tl2 = tiledlayout(4,2,'TileSpacing','compact','Padding','compact');
sgtitle(tl2,'Road Displacement and Rate');
labs2 = {'z_{LF} [m]','\dot{z}_{LF} [m/s]', ...
         'z_{RF} [m]','\dot{z}_{RF} [m/s]', ...
         'z_{LR} [m]','\dot{z}_{LR} [m/s]', ...
         'z_{RR} [m]','\dot{z}_{RR} [m/s]'};
sig2  = {zLF,dzLF,zRF,dzRF,zLR,dzLR,zRR,dzRR};
for k=1:8
    nexttile; plot(t, sig2{k}, 'LineWidth', 1); grid on;
    xlabel('Time [s]'); title(labs2{k});
end

% ====== Figure 3: Trailer body states (pos, euler, vB, wB) ===============
figure('Name','Trailer Body States','NumberTitle','off','Color','w');
tl3 = tiledlayout(4,3,'TileSpacing','compact','Padding','compact');
sgtitle(tl3,'Body States: position, attitude, linear & angular velocities');

y3 = { rB(:,1), rB(:,2), rB(:,3), ...
       phiBd, thetaBd, psiBd, ...
       vB(:,1), vB(:,2), vB(:,3), ...
       wB(:,1), wB(:,2), wB(:,3) };
nm3 = {'x_B [m]','y_B [m]','z_B [m]', ...
       '\phi_B [deg]','\theta_B [deg]','\psi_B [deg]', ...
       'u_B [m/s]','v_B [m/s]','w_B [m/s]', ...
       'p_B [rad/s]','q_B [rad/s]','r_B [rad/s]'};
for k=1:12
    nexttile; plot(t, y3{k}, 'LineWidth', 1.1); grid on;
    xlabel('Time [s]'); title(nm3{k});
end

% ======== Figure 4: Front axle outputs (pos, euler) ======================
figure('Name','Front Axle States','NumberTitle','off','Color','w');
tl4 = tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
sgtitle(tl4,'Front Axle (6-DOF)');
y4 = { rAF(:,1), rAF(:,2), rAF(:,3), phiAFd, thetaAFd, psiAFd };
nm4= { 'x_{AF} [m]','y_{AF} [m]','z_{AF} [m]', ...
       '\phi_{AF} [deg]','\theta_{AF} [deg]','\psi_{AF} [deg]'};
for k=1:6
    nexttile; plot(t, y4{k}, 'LineWidth', 1.1); grid on;
    xlabel('Time [s]'); title(nm4{k});
end

% ========= Figure 5: Rear axle outputs (pos, euler) ======================
figure('Name','Rear Axle States','NumberTitle','off','Color','w');
tl5 = tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
sgtitle(tl5,'Rear Axle (6-DOF)');
y5 = { rAR(:,1), rAR(:,2), rAR(:,3), phiARd, thetaARd, psiARd };
nm5= { 'x_{AR} [m]','y_{AR} [m]','z_{AR} [m]', ...
       '\phi_{AR} [deg]','\theta_{AR} [deg]','\psi_{AR} [deg]'};
for k=1:6
    nexttile; plot(t, y5{k}, 'LineWidth', 1.1); grid on;
    xlabel('Time [s]'); title(nm5{k});
end
end

% -------- helpers --------
function y = fun_or_zero(P, field, t)
if isfield(P, field) && ~isempty(P.(field)) && isa(P.(field),'function_handle')
    y = P.(field)(t);
else
    y = zeros(size(t));
end
y = y(:);
end
