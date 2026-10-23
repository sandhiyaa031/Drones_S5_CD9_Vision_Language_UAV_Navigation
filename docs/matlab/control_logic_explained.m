%% CONTROL LOGIC, MATHS AND PHYSICS OF THE UAV RESCUE SYSTEM  (explained in MATLAB)
% One figure per section: LaTeX equations on the left, a simulation of that
% exact piece of logic on the right. Run the whole file, or run it section
% by section (Ctrl+Enter inside a %% block).
%
% WHAT IS PROJECT CODE AND WHAT IS ILLUSTRATION  (say this out loud to the examiner)
%   [PROJECT]  numbers/equations copied from the repository and checked
%              against it: flight_controller.py, flight_safety.py,
%              avoidance_interface.py, debris_avoidance.py, debris_risk.py,
%              and PX4's x500 model file.
%   [BACKGROUND] textbook physics/control used to explain what PX4 does
%              inside. The application code itself does NOT implement PID:
%              it sends position setpoints and PX4 closes the fast loops.
%   [ILLUSTRATION] my own simple simulation (planar cascade, PID demo,
%              synthetic crash data). Gains are chosen to be well damped,
%              they are not PX4's gains and not measured results.
%
% SECTIONS
%   1 Rigid-body physics and thrust budget      6 Speed cap on the setpoint
%   2 Tilt -> horizontal acceleration           7 PID: what P, I and D do
%   3 PX4-style cascade (planar illustration)   8 Supervisory state machine + gates
%   4 Project's fitted response model           9 Safety monitor (failsafe maths)
%   5 (sec. 4 numbers vs the toy cascade)      10 Debris risk -> 11 Avoidance decision
%                                              12 One-page cheat sheet
%
% Requires base MATLAB R2019b or newer (no toolboxes). Set exportFigures=true
% to also write PNGs to ./figs.

clear; clc; close all; rng(1);
exportFigures = false;
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
outDir = fullfile(here, 'figs');
if exportFigures && ~exist(outDir, 'dir'), mkdir(outDir); end

%% 0  PARAMETERS  [PROJECT] -------------------------------------------------
P.g   = 9.80665;                       % m/s^2
P.m   = 2.0 + 4*0.016076923;           % kg: x500_base 2.0 kg + 4 rotor bodies (PX4 SDF; cameras not included)
P.Ixx = 0.02166667; P.Iyy = 0.02166667; P.Izz = 0.04;   % kg m^2, base link (PX4 SDF)
P.kf  = 8.54858e-6;                    % N/(rad/s)^2   motorConstant
P.wmax = 1000;                         % rad/s         maxRotVelocity
P.kmom = 0.016;                        % m             momentConstant (moment per thrust)
P.arm = 0.174;                         % m             rotor offset in x and y (SDF)
P.W   = P.m*P.g;                       % weight [N]
P.Tmax = 4*P.kf*P.wmax^2;              % N, all four rotors at limit

% Response model fitted to recorded PX4 SITL flights (debris_avoidance.py, AvoidanceParams)
R.tau_xy = 0.493; R.kp_xy = 0.827; R.kv_xy = 3.203; R.a_xy_max = 11.08; R.v_xy_max = 12.0;
R.tau_z  = 0.02;  R.kp_z  = 1.181; R.kv_z  = 12.0;  R.a_z_max  = 6.75;
R.v_up_max = 3.0; R.v_down_max = 1.5;  % NED: z positive DOWN, so descent is positive
R.dt = 0.02; R.horizon = 2.0; R.model_error = 0.25;

% Cost weights and limits of the avoidance planner (AvoidanceParams / debris_risk.py)
A.w_risk = 10; A.w_dist = 1.0; A.w_energy = 0.5; A.w_smooth = 0.5; A.w_alt = 0.5;
A.risk_scale = 1.5; A.min_alt = 1.5; A.max_alt = 12; A.max_vxy = 5.0; A.max_vz = 2.5;
A.uav_radius = 0.40; A.debris_factor = 0.71; A.margin = 0.30; A.reaction = 0.75;
A.cap_base = 0.10; A.cap_rate = 0.40;

PX4_XY_P = 0.95;                       % MPC_XY_P [1/s], PX4 default, unchanged by the project
blue = [0.10 0.30 0.60]; red = [0.80 0.10 0.10]; grn = [0.10 0.55 0.20]; gry = [0.45 0.45 0.45];

%% 1  RIGID-BODY PHYSICS AND THRUST BUDGET  [BACKGROUND + PROJECT numbers] ----
f = newfig('1 Rigid-body physics and thrust budget');
wHover = sqrt(P.W/(4*P.kf));
eqpanel([0.02 0.04 0.50 0.92], { ...
 'H:Newton + Euler for a quadrotor (NED: x north, y east, z DOWN)', ...
 'm\,\ddot{\mathbf{p}} = m g\,\mathbf{e}_z - T\,\mathbf{R}\,\mathbf{e}_z', ...
 'N:Gravity pulls along +z (down). Thrust T acts along the body -z axis, rotated into the local|frame by R (body-to-local rotation, R''R = I).', ...
 'T = \sum_{i=1}^{4} k_f\,\omega_i^{2}, \qquad M_{z,i} = \pm k_{mom}\,k_f\,\omega_i^{2}', ...
 'N:Each rotor makes thrust proportional to speed squared; its reaction torque is a fixed|fraction (momentConstant) of its thrust, sign set by spin direction.', ...
 '\mathbf{I}\,\dot{\mathbf{\Omega}} + \mathbf{\Omega}\times\mathbf{I}\,\mathbf{\Omega} = \tau', ...
 'N:Rotation: body rates Omega change with the torques tau from unequal rotor thrusts.|With rotors at (+-l, +-l), l = 0.174 m:  tau_roll = l (T_left - T_right) (and similarly pitch).', ...
 'H:Hover and thrust budget (numbers from the PX4 x500 model)', ...
 'T_{hover}= m g \;\Rightarrow\; \omega_{hover}=\sqrt{\frac{m g}{4 k_f}}', ...
 sprintf('m=%.3f\\,\\mathrm{kg},\\quad W=%.2f\\,\\mathrm{N},\\quad T_{max}=4k_f\\omega_{max}^2=%.1f\\,\\mathrm{N}', P.m, P.W, P.Tmax), ...
 sprintf('\\frac{T_{max}}{W}=%.2f,\\qquad \\omega_{hover}=%.0f\\,\\mathrm{rad/s}\\;(%.0f\\%%\\ \\mathrm{of\\ max\\ thrust})', P.Tmax/P.W, wHover, 100*P.W/P.Tmax), ...
 'N:Why it matters: the application commands POSITION. Whether the drone can follow is set|by this thrust margin and by PX4''s inner loops, not by project code.' ...
 }, 11);
ax = axes('Position', [0.57 0.55 0.40 0.38]); hold on; grid on; box on;
w = linspace(0, P.wmax, 200); plot(w, P.kf*w.^2, 'Color', blue, 'LineWidth', 2);
plot(wHover, P.kf*wHover^2, 'o', 'MarkerFaceColor', grn, 'MarkerEdgeColor', 'k', 'MarkerSize', 9);
yline(P.W/4, ':', 'Color', grn); plot(P.wmax, P.kf*P.wmax^2, 's', 'MarkerFaceColor', red, 'MarkerEdgeColor', 'k', 'MarkerSize', 9);
xlabel('rotor speed \omega  [rad/s]'); ylabel('thrust per rotor  [N]');
title('T_i = k_f \omega_i^2   (green: hover, red: limit)'); xlim([0 P.wmax]);
ax = axes('Position', [0.57 0.08 0.40 0.34]); hold on; grid on; box on;
bar(categorical({'weight W','max thrust'}), [P.W P.Tmax], 0.5, 'FaceColor', [0.6 0.75 0.95]);
ylabel('force  [N]'); title(sprintf('thrust-to-weight = %.2f  (margin for manoeuvres)', P.Tmax/P.W));
savefig_if(f, exportFigures, outDir, 'sec01_physics');

%% 2  TILT -> HORIZONTAL ACCELERATION  [BACKGROUND, checked against project numbers] ---
f = newfig('2 Tilt to horizontal acceleration');
th = deg2rad(linspace(0, 70, 300));
ah = P.g*tan(th); TW = 1./cos(th);
thLim = acos(P.W/P.Tmax); ahLim = P.g*tan(thLim);
thFit = atan(R.a_xy_max/P.g);
eqpanel([0.02 0.04 0.50 0.92], { ...
 'H:Holding altitude while tilting by an angle theta', ...
 'T\cos\theta = m g \quad(\mathrm{vertical\ balance}),\qquad m a_h = T\sin\theta', ...
 'a_h = g\tan\theta,\qquad \frac{T}{W}=\frac{1}{\cos\theta}', ...
 'N:To accelerate sideways the drone must FIRST rotate, then thrust redirects. That is why|horizontal response lags a position command (a first-order lag tau in the response model).', ...
 'H:What the thrust limit allows', ...
 sprintf('\\theta_{max}=\\arccos\\frac{W}{T_{max}}=%.1f^{\\circ}\\;\\Rightarrow\\; a_{h,max}=g\\tan\\theta_{max}=%.1f\\,\\mathrm{m/s^2}', rad2deg(thLim), ahLim), ...
 sprintf('\\mathrm{project\\ fitted\\ limit}\\; a_{xy,max}=%.2f\\,\\mathrm{m/s^2}\\;\\Leftrightarrow\\;\\theta=%.1f^{\\circ},\\; T/W=%.2f', R.a_xy_max, rad2deg(thFit), 1/cos(thFit)), ...
 'N:Check: the fitted acceleration limit (from recorded PX4 flights) is BELOW what the|airframe could physically do, so the response model is physically plausible.|(Ignores rotor lag, drag, and PX4''s own tilt/jerk limits.)', ...
 'N:Worked example: tilt 20 deg  ->  a_h = g tan 20 = 3.57 m/s^2, thrust = W / cos 20 = 1.06 W.' ...
 }, 11);
ax = axes('Position', [0.57 0.55 0.40 0.38]); hold on; grid on; box on;
plot(rad2deg(th), ah, 'Color', blue, 'LineWidth', 2); yline(R.a_xy_max, '--', 'fitted a_{xy,max}', 'Color', red);
xline(rad2deg(thLim), ':', 'Color', gry); xlabel('tilt \theta  [deg]'); ylabel('a_h = g tan\theta  [m/s^2]'); title('horizontal acceleration vs tilt');
ax = axes('Position', [0.57 0.08 0.40 0.34]); hold on; grid on; box on;
plot(rad2deg(th), TW, 'Color', blue, 'LineWidth', 2); yline(P.Tmax/P.W, '--', 'T_{max}/W', 'Color', red);
xlabel('tilt \theta  [deg]'); ylabel('T / W'); title('thrust needed to keep altitude'); ylim([1 3]);
savefig_if(f, exportFigures, outDir, 'sec02_tilt');

%% 3  PX4-STYLE CASCADE  [BACKGROUND: planar ILLUSTRATION, not PX4's gains] ------
f = newfig('3 Cascaded control (PX4 architecture, planar illustration)');
dt = 1e-3; T = 8; N = round(T/dt); t = (0:N)'*dt;
Kp = PX4_XY_P; Kvp = 3.2; Kvi = 0.6; Kth = 9; Kw = 40; thmax = deg2rad(35); axmax = 8;
x = 0; z = 0; thv = 0; vx = 0; vz = 0; om = 0; ix = 0;
X = zeros(N+1, 5); X(1,:) = [x vx thv 0 0];
for k = 1:N
    xsp = 4*(t(k) >= 0.5);                      % 4 m step at t = 0.5 s (north)
    vxsp = max(min(Kp*(xsp - x), 5), -5);        % outer loop: P on position -> velocity setpoint
    ex = vxsp - vx; ix = ix + ex*dt;
    axsp = max(min(Kvp*ex + Kvi*ix, axmax), -axmax);   % velocity loop (PI) -> acceleration setpoint
    vzsp = min(max(R.kp_z*(0 - z), -R.v_down_max), R.v_up_max);   % altitude hold, z UP in this planar demo
    azsp = max(min(R.kv_z*(vzsp - vz), R.a_z_max), -R.a_z_max);
    thsp = max(min(atan2(axsp, P.g + azsp), thmax), -thmax);      % thrust direction from desired acceleration
    Tc = min(max(P.m*(P.g + azsp)/cos(thv), 0), P.Tmax);          % collective thrust keeps the vertical balance
    wsp = Kth*(thsp - thv);                      % attitude loop (P) -> body-rate setpoint
    tau = P.Iyy*Kw*(wsp - om);                   % rate loop (P) -> torque
    axp = Tc/P.m*sin(thv); azp = Tc/P.m*cos(thv) - P.g; ath = tau/P.Iyy;   % plant
    om = om + ath*dt; thv = thv + om*dt; vx = vx + axp*dt; vz = vz + azp*dt; x = x + vx*dt; z = z + vz*dt;
    X(k+1,:) = [x vx thv azsp Tc];
end
[tf, pf, vf] = respSim([0 0 0], [0 0 0], [4 0 0], R, [0 0 0]);   % project's fitted model for the same 4 m step
eqpanel([0.02 0.04 0.48 0.92], { ...
 'H:Cascade (each loop is faster than the one outside it)', ...
 '\mathbf{v}_{sp}=K_p(\mathbf{p}_{sp}-\mathbf{p}),\qquad K_p=\mathrm{MPC\_XY\_P}=0.95\ \mathrm{s^{-1}}', ...
 '\mathbf{a}_{sp}=K_{vp}(\mathbf{v}_{sp}-\mathbf{v})+K_{vi}\!\int(\mathbf{v}_{sp}-\mathbf{v})\,dt', ...
 '\theta_{sp}=\arctan\frac{a_{x,sp}}{g+a_{z,sp}},\qquad T=\frac{m\,(g+a_{z,sp})}{\cos\theta}', ...
 '\dot\theta_{sp}=K_\theta(\theta_{sp}-\theta),\qquad \tau = I_{yy}K_\omega(\dot\theta_{sp}-\dot\theta)', ...
 'H:Plant (planar x-z-theta, z up in this illustration)', ...
 '\ddot x=\frac{T}{m}\sin\theta,\qquad \ddot z=\frac{T}{m}\cos\theta-g,\qquad \ddot\theta=\frac{\tau}{I_{yy}}', ...
 'N:What the PROJECT does: it only chooses the position setpoint p_sp and streams it at 20 Hz.|Everything below the setpoint is PX4. Here: K_p = 0.95 is PX4''s default; the other gains|are mine, chosen only to be well damped.', ...
 'N:Orange curve: the project''s FITTED model of PX4 (section 4) for the same step. It lags more:|its tau = 0.49 s is an empirical fit to recorded PX4-in-simulation flights, not derived from this toy.' ...
 }, 11);
ax = axes('Position', [0.55 0.70 0.42 0.24]); hold on; grid on; box on;
plot(t, 4*(t >= 0.5), 'k--'); plot(t, X(:,1), 'Color', blue, 'LineWidth', 2); plot(tf + 0.5, pf(:,1), 'Color', [0.9 0.5 0.1], 'LineWidth', 2);
ylabel('north x [m]'); legend('setpoint', 'toy cascade', 'project fitted model', 'Location', 'southeast'); xlim([0 6]);
ax = axes('Position', [0.55 0.39 0.42 0.24]); hold on; grid on; box on;
plot(t, X(:,2), 'Color', blue, 'LineWidth', 2); plot(tf + 0.5, vf(:,1), 'Color', [0.9 0.5 0.1], 'LineWidth', 2); ylabel('speed v_x [m/s]'); xlim([0 6]);
ax = axes('Position', [0.55 0.08 0.42 0.24]); hold on; grid on; box on;
plot(t, rad2deg(X(:,3)), 'Color', blue, 'LineWidth', 2); ylabel('pitch \theta [deg]'); xlabel('time [s]'); xlim([0 6]);
title('the drone tilts first, then moves: command is not motion');
savefig_if(f, exportFigures, outDir, 'sec03_cascade');

%% 4  THE PROJECT'S FITTED RESPONSE MODEL  [PROJECT: exact port of simulate_response] ---
f = newfig('4 Response model used by the avoidance planner');
d45 = 1/sqrt(2);
[ts, p4, v4, a4] = respSim([0 0 0], [0 0 0], 4*[d45 -d45 0], R, [0 0 0]);   % 4 m north-west from hover
[~, p25, ~, ~]   = respSim([0 0 0], [0 0 0], 2.5*[d45 -d45 0], R, [0 0 0]);
[~, pz, vz4, ~]  = respSim([0 0 0], [0 0 0], [0 0 -2], R, [0 0 0]);          % 2 m climb (NED z negative)
off4 = vecnorm(p4(:,1:2), 2, 2); spd4 = vecnorm(v4(:,1:2), 2, 2);
tt = [0.5 1.0 1.068 1.5 2.0]; idx = round(tt/R.dt) + 1;
fprintf('\n[4] 4 m north-west sidestep from hover (project response model):\n   t [s]   offset [m]   speed [m/s]\n');
for k = 1:numel(tt), fprintf('   %.3f    %.2f         %.2f\n', tt(k), off4(idx(k)), spd4(idx(k))); end
eqpanel([0.02 0.04 0.50 0.92], { ...
 'H:Reduced model of PX4''s cascade (per axis, integrated every 20 ms for 2 s)', ...
 '\mathbf{v}_{cmd}=\mathrm{clamp}\big(k_p(\mathbf{p}_{sp}-\mathbf{p}),\,v_{max}\big)', ...
 '\mathbf{a}_{cmd}=\mathrm{clamp}\big(k_v(\mathbf{v}_{cmd}-\mathbf{v}),\,a_{max}\big)', ...
 '\mathbf{a}_{k+1}=\mathbf{a}_k+\min\!\left(1,\frac{\Delta t}{\tau}\right)(\mathbf{a}_{cmd}-\mathbf{a}_k)', ...
 '\mathbf{p}_{k+1}=\mathbf{p}_k+\mathbf{v}_k\Delta t+\frac12\mathbf{a}_{k+1}\Delta t^2,\qquad \mathbf{v}_{k+1}=\mathbf{v}_k+\mathbf{a}_{k+1}\Delta t', ...
 'N:Horizontal: tau = 0.493 s, k_p = 0.827, k_v = 3.203, a_max = 11.08.  Vertical: tau = 0.02 s,|k_p = 1.181, k_v = 12, a_max = 6.75, climb <= 3.0, descent <= 1.5 m/s (z is down).', ...
 'N:Fitted to 133 recorded start states; largest position error up to 1 s ahead was 0.25 m,|which is added to every clearance requirement as "model_error".', ...
 'H:Why a late warning is fatal  (4 m sidestep from hover)', ...
 sprintf('\\mathrm{offset}(0.5\\,\\mathrm{s})=%.2f\\,\\mathrm{m},\\quad \\mathrm{offset}(1.0\\,\\mathrm{s})=%.2f\\,\\mathrm{m},\\quad \\mathrm{offset}(1.07\\,\\mathrm{s})=%.2f\\,\\mathrm{m}', off4(idx(1)), off4(idx(2)), off4(idx(3))), ...
 'N:A box that arrives 1.07 s after the warning finds the drone only ~1.9 m from where it was.|This is the reason for the measured ~0.9 s MINIMUM useful warning.' ...
 }, 11);
ax = axes('Position', [0.57 0.70 0.40 0.24]); hold on; grid on; box on;
plot(ts, off4, 'Color', blue, 'LineWidth', 2); plot(ts, vecnorm(p25(:,1:2), 2, 2), 'Color', grn, 'LineWidth', 2);
xline(1.068, '--', 't = 1.07 s', 'Color', red); ylabel('horizontal offset [m]'); legend('4 m setpoint', '2.5 m setpoint', 'Location', 'northwest');
ax = axes('Position', [0.57 0.39 0.40 0.24]); hold on; grid on; box on;
plot(ts, spd4, 'Color', blue, 'LineWidth', 2); plot(ts, vecnorm(a4(:,1:2), 2, 2)/3, 'Color', [0.9 0.5 0.1], 'LineWidth', 1.5);
yline(A.max_vxy, ':', 'speed limit 5 m/s', 'Color', red); ylabel('speed [m/s]  (orange: |a|/3)');
ax = axes('Position', [0.57 0.08 0.40 0.24]); hold on; grid on; box on;
plot(ts, -pz(:,3), 'Color', blue, 'LineWidth', 2); ylabel('climb (2 m setpoint) [m]'); xlabel('time [s]');
savefig_if(f, exportFigures, outDir, 'sec04_response_model');

%% 5  SPEED CAP ON THE SETPOINT  [PROJECT: avoidance_interface.limit_setpoint] ----
f = newfig('5 Speed cap by limiting how far ahead the setpoint is');
eqpanel([0.02 0.04 0.50 0.92], { ...
 'H:PX4''s position loop commands velocity = K_p x error', ...
 'v_{cmd}=K_p\,e,\qquad K_p=0.95\ \mathrm{s^{-1}}', ...
 'N:So the project does not touch PX4''s gains. To cap the speed at v_lim it only keeps the|setpoint no further than "reach" from the vehicle:', ...
 '\mathrm{reach}=\frac{v_{lim}}{K_p},\qquad \mathbf{p}_{sp}\leftarrow\mathbf{p}+\hat{\mathbf{e}}\,\min(\left\|\mathbf{e}\right\|,\ \mathrm{reach})', ...
 sprintf('v_{lim}=2\\,\\mathrm{m/s}\\Rightarrow \\mathrm{reach}=%.2f\\,\\mathrm{m};\\qquad \\mathrm{error}\\ 4\\,\\mathrm{m}\\Rightarrow v=%.1f\\,\\mathrm{m/s\\ if\\ uncapped}', 2/PX4_XY_P, PX4_XY_P*4), ...
 'N:Used for: search legs (cap 2 m/s), degraded or stale perception (2.0 / 1.0 m/s), and the|search planner''s requested speed. A cap can only restrict, never speed the vehicle up.', ...
 'N:Limitation: it caps the commanded velocity, not acceleration. A hard start can still|overshoot the cap briefly (hard-acceleration transit is a documented failure case).' ...
 }, 11);
ax = axes('Position', [0.57 0.55 0.40 0.38]); hold on; grid on; box on;
e = linspace(0, 12, 200);
plot(e, PX4_XY_P*e, 'k--', 'LineWidth', 1.2);
for vl = [1 2 4]
    reach = vl/PX4_XY_P; plot(e, PX4_XY_P*min(e, reach), 'LineWidth', 2);
end
xlabel('distance from vehicle to mission target [m]'); ylabel('commanded speed [m/s]');
legend('uncapped (K_p e)', 'cap 1 m/s', 'cap 2 m/s', 'cap 4 m/s', 'Location', 'northwest'); title('speed command vs target distance');
ax = axes('Position', [0.57 0.08 0.40 0.34]); hold on; grid on; box on; axis equal;
pos0 = [0 0]; tgt = [10 4]; [sx, sy] = limitSetpoint(pos0, tgt, 2.0, PX4_XY_P);
plot([0 tgt(1)], [0 tgt(2)], 'k:'); plot(tgt(1), tgt(2), 'rs', 'MarkerFaceColor', red); plot(sx, sy, 'o', 'MarkerFaceColor', grn, 'MarkerEdgeColor', 'k', 'MarkerSize', 9);
plot(0, 0, 'b^', 'MarkerFaceColor', blue); xlabel('north [m]'); ylabel('east [m]');
title(sprintf('target (10, 4) -> setpoint sent to PX4 (%.2f, %.2f)', sx, sy)); legend('line to target', 'mission target', 'capped setpoint', 'vehicle');
savefig_if(f, exportFigures, outDir, 'sec05_speedcap');

%% 6  PID: WHAT P, I AND D DO  [BACKGROUND: PX4 inner loops, not project code] ---
f = newfig('6 PID control (inside PX4, not in the application)');
dtp = 1e-3; Tp = 6; Np = round(Tp/dtp); tp = (0:Np)'*dtp; mass = 1; b = 0.5; dist = -3;  % constant disturbance force
gains = [20 0 0; 20 15 0; 20 15 8];  names = {'P only', 'PI', 'PID'}; Y = zeros(Np+1, 3); parts = zeros(Np+1, 3);
for c = 1:3
    y = 0; v = 0; I = 0; Kp_ = gains(c,1); Ki_ = gains(c,2); Kd_ = gains(c,3);
    for k = 1:Np
        e_ = 1 - y; I = I + e_*dtp; u = Kp_*e_ + Ki_*I - Kd_*v;      % derivative on measurement (no set-point kick)
        v = v + (u - b*v + dist)/mass*dtp; y = y + v*dtp; Y(k+1,c) = y;
        if c == 3, parts(k+1,:) = [Kp_*e_, Ki_*I, -Kd_*v]; end
    end
end
eqpanel([0.02 0.04 0.46 0.92], { ...
 'H:PID law', ...
 'u(t)=K_P\,e(t)+K_I\int_0^t e(\lambda)\,d\lambda+K_D\,\frac{de(t)}{dt},\qquad e=y_{sp}-y', ...
 'N:P: pushes in proportion to the present error (fast, but leaves an offset under a|constant disturbance).  I: accumulates error and removes that offset (risk: wind-up|when the actuator saturates).  D: reacts to the error''s rate and damps overshoot.', ...
 'H:Plant used in this demo (illustrative)', ...
 'm\ddot y = u - b\,\dot y + d,\qquad m=1,\ b=0.5,\ d=-3\ \mathrm{(constant\ push)}', ...
 'N:With P only, the steady error is d / K_P = 3/20 = 0.15 (see left plot). Adding I drives it|to zero. Gains here: K_P=20, K_I=15, K_D=8 (illustrative).', ...
 'H:Where it appears in the real system', ...
 'N:PX4: velocity loop (PID), attitude loop (P), body-rate loop (PID), all inside PX4.|Project code: NONE. flight_controller.py only publishes position setpoints at 20 Hz and|tests PX4''s feedback (altitude within 0.25 m, waypoint within 0.5 m, arming flags).' ...
 }, 11);
ax = axes('Position', [0.53 0.56 0.44 0.37]); hold on; grid on; box on;
plot(tp, ones(size(tp)), 'k--'); plot(tp, Y, 'LineWidth', 2); xlabel('time [s]'); ylabel('y'); legend(['setpoint' names], 'Location', 'southeast'); ylim([0.7 1.3]); title('step response under a constant disturbance');
ax = axes('Position', [0.53 0.08 0.44 0.37]); hold on; grid on; box on;
plot(tp, parts(:,1), 'LineWidth', 1.8); plot(tp, parts(:,2), 'LineWidth', 1.8); plot(tp, parts(:,3), 'LineWidth', 1.8);
xlabel('time [s]'); ylabel('control effort'); legend('K_P e', 'K_I \int e', '-K_D dy/dt'); title('contribution of each term (PID run)');
savefig_if(f, exportFigures, outDir, 'sec06_pid');

%% 7  SUPERVISORY STATE MACHINE AND ITS GATES  [PROJECT: flight_controller.py] ---
f = newfig('7 Supervisory logic: transitions are tests on PX4 feedback');
R7 = R; R7.horizon = 14;
[t7, p7, v7] = respSim([0 0 0], [0 0 0], [0 0 -5], R7, [0 0 0]);             % TAKEOFF to 5 m (NED z = -5), model of PX4
alt = -p7(:,3); iT = find(abs(alt - 5) <= 0.25, 1); tTake = t7(iT);
[t8, p8, v8] = respSim([0 0 0], [0 0 0], [8 0 0], R7, [0 0 0]);               % 8 m leg to "home"
dist8 = 8 - p8(:,1); spd8 = abs(v8(:,1));
i1 = find(dist8 <= 0.5, 1);                                                    % no arrival gate
i2 = find(dist8 <= 0.5 & spd8 <= 0.3, 1);                                      % arrival_max_speed = 0.3 m/s
if isempty(i2), i2 = numel(t8); end
fprintf('\n[7] model-based takeoff: |alt-5| <= 0.25 m after %.2f s (measured 3 m takeoffs on PX4: 6.7-7.3 s; the model is fitted to avoidance steps, not to takeoff)\n', tTake);
fprintf('    8 m leg: accepted without gate at %.2f s moving at %.2f m/s; with gate at %.2f s moving at %.2f m/s\n', t8(i1), spd8(i1), t8(i2), spd8(i2));
eqpanel([0.02 0.04 0.50 0.92], { ...
 'H:Open-loop setpoints, closed-loop LOGIC (20 Hz timer, 0.05 s tick)', ...
 'N:WAIT_FOR_DATA -> PRESTREAM (1.5 s of setpoints so PX4 accepts Offboard) -> REQUEST_OFFBOARD|-> WAIT_PREFLIGHT -> REQUEST_ARM -> TAKEOFF -> SEARCH / HOLD / WAYPOINTS -> RETURN_HOME|-> REQUEST_LAND -> LANDING -> COMPLETE;  any violation -> FAILSAFE -> PX4 AUTO LAND.', ...
 'H:Transition tests (all on PX4 feedback)', ...
 '\mathrm{takeoff\ done:}\quad |h-h_{target}|\le 0.25\ \mathrm{m}', ...
 '\mathrm{waypoint\ reached:}\quad \sqrt{e_N^2+e_E^2}\le 0.5\ \mathrm{m},\quad |h-h_{exp}|\le 0.35\ \mathrm{m}', ...
 '\mathrm{(optional\ gate)}\quad \left\|\mathbf{v}_{xy}\right\|\le v_{arr}\quad (\mathrm{e.g.}\ 0.3\ \mathrm{m/s})', ...
 'N:Timeouts: takeoff 30 s, waypoint 45 s, landing 45 s, preflight 30 s, telemetry age 1 s. A|timeout is a failure -> FAILSAFE. Altitude hold must stay within 0.35 m for 30 s.', ...
 'H:Why the arrival-speed gate exists  (8 m leg, model)', ...
 sprintf('\\mathrm{no\\ gate:\\ accepted\\ at\\ %.1f\\,s\\ while\\ moving\\ %.2f\\,m/s;\\quad gate:\\ %.1f\\,s\\ at\\ %.2f\\,m/s}', t8(i1), spd8(i1), t8(i2), spd8(i2)), ...
 'N:Without the gate "home reached" fires while flying past home, and PX4 AUTO LAND then drifts.|Measured in the PX4-in-MuJoCo loop: landing offset 2.1-2.2 m without the gate, 0.35 m with it.' ...
 }, 11);
ax = axes('Position', [0.57 0.55 0.40 0.38]); hold on; grid on; box on;
plot(t7, alt, 'Color', blue, 'LineWidth', 2); yline(5, 'k--'); patch([0 14 14 0], [4.75 4.75 5.25 5.25], grn, 'FaceAlpha', 0.15, 'EdgeColor', 'none');
xline(tTake, ':', sprintf('TAKEOFF -> next state at %.1f s', tTake), 'Color', red, 'LabelOrientation', 'horizontal');
xlabel('time since arming [s]'); ylabel('altitude [m]'); title('takeoff target 5 m, tolerance +-0.25 m (model of PX4)');
ax = axes('Position', [0.57 0.08 0.40 0.36]); hold on; grid on; box on;
yyaxis left; plot(t8, dist8, 'Color', blue, 'LineWidth', 2); yline(0.5, ':', 'Color', blue); ylabel('distance to home [m]');
yyaxis right; plot(t8, spd8, 'Color', red, 'LineWidth', 2); yline(0.3, ':', 'Color', red); ylabel('speed [m/s]');
xline(t8(i1), '--', 'no gate'); xline(t8(i2), '-', 'with gate'); xlabel('time [s]'); title('arrival test: distance <= 0.5 m, with and without speed gate');
savefig_if(f, exportFigures, outDir, 'sec07_state_machine');

%% 8  SAFETY MONITOR  [PROJECT: flight_safety.py, tests on every PX4 position sample] ---
f = newfig('8 Safety monitor: when the fallback fires');
fs = 100; dts = 1/fs; ts8 = (0:dts:6)'; n8 = numel(ts8);
vx8 = 2*ones(n8,1); tImp = 4.00; vx8(ts8 >= tImp) = 0; vx8(ts8 >= tImp & ts8 < tImp + 0.03) = 2*(1 - (ts8(ts8 >= tImp & ts8 < tImp + 0.03) - tImp)/0.03);
ax8 = [0; diff(vx8)/dts]; ax8 = ax8 + 0.3*randn(n8,1).*(ts8 > 0.1); aN = abs(ax8);
x8 = cumsum(vx8)*dts;
dtv = linspace(0.005, 0.2, 200); thrP = 0.5 + 0.5*30*dtv.^2; thrV = 3.0 + 30*dtv;
% scenario B: estimator reset -> position jumps +1.2 m without matching velocity
xj = x8; xj(ts8 >= 2.5) = xj(ts8 >= 2.5) + 1.2; resid = zeros(n8,1);
for k = 2:n8, resid(k) = abs((xj(k) - xj(k-1)) - 0.5*(vx8(k) + vx8(k-1))*dts); end
iImp = find(aN > 30, 1); iJmp = find(resid > 0.5 + 0.5*30*dts^2, 1);
fprintf('\n[8] impact detected at t = %.2f s (event at %.2f s, |a| = %.0f m/s^2); position-jump detected at t = %.2f s (residual %.2f m > %.3f m)\n', ts8(iImp), tImp, aN(iImp), ts8(iJmp), resid(iJmp), 0.5 + 0.5*30*dts^2);
eqpanel([0.02 0.04 0.48 0.92], { ...
 'H:Checks on every position sample while airborne (> 0.5 m)', ...
 '\mathrm{impact:}\quad \left\|\mathbf{a}\right\|>30\ \mathrm{m/s^2}', ...
 '\mathrm{speed\ envelope:}\quad v_{xy}>6.5\ \mathrm{m/s}\ \ \mathrm{or}\ \ |v_z|>3.0\ \mathrm{m/s}', ...
 '\mathrm{position\ jump:}\quad \left\|\Delta\mathbf{p}-\bar{\mathbf{v}}\,\Delta t\right\| > 0.5+\frac12\,(30)\,\Delta t^2', ...
 '\mathrm{velocity\ jump:}\quad \left\|\Delta\mathbf{v}\right\| > 3.0+30\,\Delta t', ...
 'N:Position jump compares the measured step with what the mean velocity v-bar explains. The|slack 0.5 (30) dt^2/2 is the most real motion below the impact threshold could add.|Also: estimator reset counters changed, non-finite value, altitude excursion > 1 m in cruise.', ...
 'H:Stale telemetry', ...
 '\mathrm{age(position)}>1.0\ \mathrm{s}\ \ \mathrm{or}\ \ \mathrm{age(status)}>T_{status}\ \ (1.0\ \mathrm{s};\ 3.0\ \mathrm{s\ in\ urban\ runs})', ...
 'N:PX4 publishes status at ~2 Hz of SIMULATION time, so at real-time factor 0.5 two messages|are ~1 s apart in wall time: that is why the urban runs use 3 s (position keeps 1 s).', ...
 'H:Action', ...
 '\mathrm{violation\ latched}\ \Rightarrow\ \mathrm{FAILSAFE}\ \Rightarrow\ \mathrm{NAV\_LAND\ (PX4\ AUTO\ LAND)}', ...
 'N:Needs no perception, planner or map: the last layer that is certain to act.' ...
 }, 11);
ax = axes('Position', [0.55 0.72 0.42 0.22]); hold on; grid on; box on;
plot(ts8, aN, 'Color', blue); yline(30, '--', '30 m/s^2 impact limit', 'Color', red); xline(ts8(iImp), ':', 'detected', 'Color', red);
ylabel('|a| [m/s^2]'); title('synthetic crash: 2 m/s -> 0 in 30 ms at t = 4 s (illustration)'); xlim([3 5]);
ax = axes('Position', [0.55 0.41 0.42 0.22]); hold on; grid on; box on;
plot(ts8, resid, 'Color', blue); yline(0.5 + 0.5*30*dts^2, '--', 'jump limit', 'Color', red); ylabel('residual [m]'); title('synthetic estimator reset: +1.2 m step at t = 2.5 s'); xlim([2 3]);
ax = axes('Position', [0.55 0.08 0.42 0.24]); hold on; grid on; box on;
plot(dtv*1000, thrP, 'Color', blue, 'LineWidth', 2); plot(dtv*1000, thrV, 'Color', grn, 'LineWidth', 2);
xlabel('sample gap \Delta t [ms]'); ylabel('threshold'); legend('position [m]: 0.5+15\Delta t^2', 'velocity [m/s]: 3+30\Delta t', 'Location', 'northwest'); title('jump thresholds grow with the sample gap');
savefig_if(f, exportFigures, outDir, 'sec08_safety');

%% 9  DEBRIS COLLISION RISK  [PROJECT: debris_risk.py; toy numbers are ILLUSTRATIVE] ---
f = newfig('9 Collision risk: ballistic prediction and closest approach');
r0 = [0.3 0 -12]; v0 = [0 0 6]; acc = [0 0 9.8];                      % box 12 m above, 0.3 m north, falling 6 m/s (NED, z down)
th_ = (0:0.01:2)'; rb = r0 + v0.*th_ + 0.5*acc.*th_.^2; dB = vecnorm(rb, 2, 2);
[dmin, im] = min(dB); tca = th_(im);
size_ = 0.4; contact = A.uav_radius + A.debris_factor*size_; safety = contact + A.margin;
rcv = r0 + v0.*th_; dCV = vecnorm(rcv, 2, 2);                          % constant-velocity (no gravity) prediction
tstar = min(max(-dot(r0, v0)/dot(v0, v0), 0), 2); dstar = norm(r0 + v0*tstar);
margin = min(1.0, A.cap_base + A.cap_rate*tca);                         % sigma assumed wide: the cap binds
cons = dmin - margin;
if dmin < contact || (dmin < safety && tca <= A.reaction), st = 'CRITICAL';
elseif dmin < safety, st = 'WARNING'; elseif cons < safety, st = 'WATCH'; else, st = 'SAFE'; end
fprintf('\n[9] ballistic: d_min = %.2f m at t_ca = %.2f s  -> %s ; constant velocity closed form: t* = %.2f s, d = %.2f m\n', dmin, tca, st, tstar, dstar);
eqpanel([0.02 0.04 0.50 0.92], { ...
 'H:Relative motion (UAV assumed to keep its velocity)', ...
 '\mathbf{r}(h)=\mathbf{p}_{deb}(t+h)-\mathbf{p}_{uav}(t+h),\qquad \mathbf{p}_{deb}(t+h)=\mathbf{p}+\mathbf{v}h+\frac12\mathbf{a}h^2,\ \ \mathbf{a}=(0,0,9.8)', ...
 'd_{min}=\min_{h\in[0,2\,\mathrm{s}]}\left\|\mathbf{r}(h)\right\|,\qquad t_{ca}=\arg\min_h\left\|\mathbf{r}(h)\right\|\ \ (\mathrm{sampled\ every\ 0.01\ s})', ...
 'N:The closed form below is computed and published for COMPARISON only; decisions use the|sampled minimum, because gravity makes the relative velocity non-constant:', ...
 '\tau^{*}=-\frac{\mathbf{r}_0\cdot\mathbf{v}_{rel}}{\mathbf{v}_{rel}\cdot\mathbf{v}_{rel}},\qquad \tau^{*}\in[0,\ \mathrm{horizon}]', ...
 'H:Radii and states', ...
 sprintf('r_{contact}=0.40+0.71\\,s=%.3f\\,\\mathrm{m},\\qquad r_{safety}=r_{contact}+0.30=%.3f\\,\\mathrm{m}\\quad (s=0.4\\,\\mathrm{m})', contact, safety), ...
 '\mathrm{CRITICAL:}\ d_{min}<r_{contact}\ \ \mathrm{or}\ \ (d_{min}<r_{safety}\ \wedge\ t_{ca}\le0.75\,\mathrm{s});\quad \mathrm{WARNING:}\ d_{min}<r_{safety}', ...
 '\mathrm{WATCH:}\ d_{min}-\min(\sigma_r,\ 0.10+0.40\,t_{ca})<r_{safety};\quad \mathrm{else\ SAFE}', ...
 'H:Illustrative case: box 12 m above, 0.3 m off the vertical, falling 6 m/s', ...
 sprintf('d_{min}=%.2f\\,\\mathrm{m}\\ \\mathrm{at}\\ t_{ca}=%.2f\\,\\mathrm{s}\\ \\Rightarrow\\ \\mathbf{%s};\\quad \\mathrm{constant\\ velocity\\ wrongly\\ says}\\ t^{*}=%.1f\\,\\mathrm{s}', dmin, tca, st, tstar), ...
 'N:Lead time (warning to impact) is only ~1.07 s here; the measured median in hover tests was 1.30 s.' ...
 }, 10.5);
ax = axes('Position', [0.57 0.55 0.40 0.38]); hold on; grid on; box on;
plot(th_, dB, 'Color', blue, 'LineWidth', 2); plot(th_, dCV, 'Color', [0.9 0.5 0.1], 'LineWidth', 2);
yline(contact, '--', 'contact', 'Color', red); yline(safety, '--', 'safety', 'Color', [0.9 0.5 0]); xline(tca, ':', 't_{ca}', 'Color', gry);
xlabel('prediction horizon h [s]'); ylabel('|r(h)| [m]'); ylim([0 14]); legend('ballistic (used)', 'constant velocity', 'Location', 'northeast'); title('separation to the falling box');
ax = axes('Position', [0.57 0.08 0.40 0.36]); hold on; grid on; box on;
plot(th_, -rb(:,3), 'Color', blue, 'LineWidth', 2); plot(th_, -rcv(:,3), 'Color', [0.9 0.5 0.1], 'LineWidth', 2); yline(0, 'k:');
xlabel('time [s]'); ylabel('box height above UAV [m]'); legend('ballistic: 12-6h-4.9h^2', 'constant velocity: 12-6h'); title('gravity matters: error 0.5 g h^2 = 5.6 m at 1.07 s');
savefig_if(f, exportFigures, outDir, 'sec09_risk');

%% 10  AVOIDANCE DECISION  [PROJECT: debris_avoidance.evaluate, simplified re-implementation] ---
f = newfig('10 Avoidance planner: 19 candidates, constraints, cost');
p0 = [0 0 -6]; groundZ = 0; mission = p0;                                    % hovering at 6 m, mission setpoint = here
boxStart = p0 + r0;                                                          % absolute box start (NED)
D = 1/sqrt(2); dirs = {'north',[1 0 0]; 'south',[-1 0 0]; 'east',[0 1 0]; 'west',[0 -1 0]; 'north_east',[D D 0]; 'north_west',[D -D 0]; 'south_east',[-D D 0]; 'south_west',[-D -D 0]};
names = {'hold'}; targets = mission;
for k = 1:size(dirs,1), for dd = [2.5 4.0], names{end+1} = sprintf('%s_%g', dirs{k,1}, dd); targets(end+1,:) = p0 + dirs{k,2}*dd; end, end %#ok<SAGROW>
names{end+1} = 'climb_2'; targets(end+1,:) = p0 + [0 0 -1]*2; names{end+1} = 'descend_1'; targets(end+1,:) = p0 + [0 0 1]*1;
C = size(targets,1);
clear_ = inf(C,1); feas = false(C,1); why = repmat({''}, C, 1); J = inf(C,1); Pend = zeros(C,3);
for c = 1:C
    [tc, pc, vc] = respSim(p0, [0 0 0], targets(c,:), R, [0 0 0]);
    pb = boxStart + v0.*tc + 0.5*acc.*tc.^2; valid = pb(:,3) <= groundZ;         % box counts while above the ground
    sep = vecnorm(pb(valid,:) - pc(valid,:), 2, 2); capm = A.cap_base + A.cap_rate*tc(valid);   % sigma assumed wide: cap binds
    clear_(c) = min(sep - safety - capm - R.model_error);
    altc = groundZ - pc(:,3); vxy = max(vecnorm(vc(:,1:2), 2, 2)); vzm = max(abs(vc(:,3)));
    if min(altc) < A.min_alt && min(altc) < altc(1) - 1e-6, why{c} = 'below min altitude';
    elseif max(altc) > A.max_alt, why{c} = 'above max altitude';
    elseif vxy > A.max_vxy && ~strcmp(names{c}, 'hold'), why{c} = 'horizontal speed limit';
    elseif vzm > A.max_vz, why{c} = 'vertical speed limit';
    elseif clear_(c) < 0, why{c} = 'safety radius + margin'; end
    feas(c) = isempty(why{c});
    risk = min(max(1 - clear_(c)/A.risk_scale, 0), 1);
    dev = norm(targets(c,:) - mission);              % anchor = mission setpoint = present position: segment is a point
    eff = norm(targets(c,:) - p0); chg = 0; altch = abs(targets(c,3) - mission(3));
    J(c) = A.w_risk*risk + A.w_dist*dev + A.w_energy*eff + A.w_smooth*chg + A.w_alt*altch;
    Pend(c,:) = pc(end,:);
end
Jf = J; Jf(~feas) = inf; [Jbest, ib] = min(Jf);
fprintf('\n[10] candidates (illustrative box, sigma assumed wide):\n');
[~, ord] = sort(J);
for c = ord(:)'
    fprintf('   %-14s clearance %+6.2f m   J = %6.2f   %s\n', names{c}, clear_(c), J(c), ifelse(feas(c), 'FEASIBLE', ['rejected: ' why{c}]));
end
if isinf(Jbest), fprintf('   -> nothing feasible: best effort (largest clearance)\n'); [~, ib] = max(clear_); else, fprintf('   -> selected: %s (J = %.2f)\n', names{ib}, Jbest); end
eqpanel([0.02 0.04 0.50 0.92], { ...
 'H:Candidate set (19)', ...
 'N:hold + 8 horizontal directions x {2.5, 4.0} m + climb 2 m + descend 1 m. Each is rolled out|through the response model of section 4 for 2 s.', ...
 'H:Hard constraints (rejected anywhere on the rollout)', ...
 '1.5\le h\le 12\ \mathrm{m},\qquad v_{xy}\le5\ \mathrm{m/s},\quad |v_z|\le2.5\ \mathrm{m/s}', ...
 'c_{lear}=\min_t\Big(\left\|\mathbf{p}_{deb}(t)-\mathbf{p}_{uav}(t)\right\|-r_{safety}-\mathrm{margin}(t)-0.25\Big)\ \ge 0', ...
 '\mathrm{margin}(t)=\min\big(\sigma_r,\ 0.10+0.40\,t\big)\qquad (\sigma_r\ \mathrm{assumed\ large\ here})', ...
 'H:Cost of a feasible candidate', ...
 'J=10\,\mathrm{risk}+1.0\,\mathrm{dev}+0.5\,\mathrm{effort}+0.5\,\mathrm{change}+0.5\,\mathrm{alt}', ...
 '\mathrm{risk}=\mathrm{clip}\!\left(1-\frac{c_{lear}}{1.5\,\mathrm{m}},\ 0,\ 1\right)', ...
 'N:The lowest-J feasible candidate becomes a PROPOSAL (AvoidanceCommand). If nothing is feasible,|the largest-clearance one is proposed only if it gains >= 0.2 m over holding ("best effort").', ...
 'H:Then the controller (the only PX4 commander) validates it', ...
 '\mathrm{age}\le0.5\,\mathrm{s},\ \ \mathrm{finite},\ \ \left\|\mathbf{p}_{sp}-\mathbf{p}\right\|\le 6\,\mathrm{m},\ \ 1.5\le h_{sp}\le15\,\mathrm{m}', ...
 'N:Simplified here: no structure keep-out boxes, no covariance (margin = its cap), one threat,|no previous setpoint. The real planner adds those.' ...
 }, 10.5);
ax = axes('Position', [0.57 0.40 0.40 0.54]); hold on; grid on; box on; axis equal;
th2 = linspace(0, 2*pi, 100); plot(0.3 + safety*cos(th2), safety*sin(th2), '-', 'Color', [0.9 0.5 0.1]);
plot(0.3, 0, 'rx', 'MarkerSize', 12, 'LineWidth', 2);
for c = 1:C
    mk = 'o'; fc = [0.8 0.8 0.8]; if feas(c), fc = grn; end
    plot(targets(c,1), targets(c,2), mk, 'MarkerFaceColor', fc, 'MarkerEdgeColor', 'k', 'MarkerSize', 7);
end
plot(targets(ib,1), targets(ib,2), 'p', 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'MarkerSize', 16);
plot(0, 0, 'b^', 'MarkerFaceColor', blue, 'MarkerSize', 9);
xlabel('north [m]'); ylabel('east [m]'); title('setpoints (green = feasible, star = chosen); red x = box column; ring = safety radius');
ax = axes('Position', [0.57 0.08 0.40 0.24]); hold on; grid on; box on;
bar(1:C, clear_, 'FaceColor', [0.6 0.75 0.95]); yline(0, 'r--'); set(gca, 'XTick', 1:C, 'XTickLabel', names, 'XTickLabelRotation', 70, 'FontSize', 7);
ylabel('clearance [m]'); title('min clearance of each candidate (>= 0 needed)');
savefig_if(f, exportFigures, outDir, 'sec10_avoidance');

%% 11  ONE-PAGE CHEAT SHEET  -------------------------------------------------
f = newfig('11 Cheat sheet: the control logic in one page');
eqpanel([0.02 0.04 0.47 0.92], { ...
 'H:Physics', ...
 'm\ddot{\mathbf{p}}=mg\,\mathbf{e}_z-T\mathbf{R}\mathbf{e}_z,\quad T=\sum k_f\omega_i^2,\quad a_h=g\tan\theta', ...
 'H:Who controls what', ...
 'N:project: proposals (goals, sidesteps)  ->  flight_controller validates, caps speed, streams|position setpoints at 20 Hz  ->  PX4: position P -> velocity PID -> attitude P -> rate PID|-> mixer -> motors.  Command is NOT motion: tilt first, then move (lag tau = 0.49 s).', ...
 'H:Speed cap', ...
 '\mathrm{reach}=v_{lim}/K_p,\quad K_p=0.95\ \mathrm{s^{-1}}', ...
 'H:Fitted response model (planner)', ...
 '\mathbf{v}_{cmd}=\mathrm{clamp}(k_p\mathbf{e}),\ \ \mathbf{a}_{cmd}=\mathrm{clamp}(k_v(\mathbf{v}_{cmd}-\mathbf{v})),\ \ \dot{\mathbf{a}}=\frac{\mathbf{a}_{cmd}-\mathbf{a}}{\tau}', ...
 'H:Risk', ...
 'd_{min}=\min_h\left\|\mathbf{r}(h)\right\|,\ \ r_{safety}=0.70+0.71s\ \ (0.40+0.71s+0.30)', ...
 'H:Avoidance', ...
 'J=10\,\mathrm{risk}+\mathrm{dev}+0.5\,\mathrm{effort}+0.5\,\mathrm{change}+0.5\,\mathrm{alt}' ...
 }, 11);
eqpanel([0.52 0.04 0.46 0.92], { ...
 'H:State machine tests', ...
 '|h-h_{tgt}|\le0.25\ (\mathrm{takeoff}),\quad \mathrm{dist}\le0.5\ (\mathrm{waypoint}),\quad v_{xy}\le v_{arr}\ (\mathrm{optional})', ...
 'H:Safety monitor', ...
 '\left\|\mathbf{a}\right\|>30,\ \ v_{xy}>6.5,\ \ |v_z|>3,\ \ \left\|\Delta\mathbf{p}-\bar{\mathbf{v}}\Delta t\right\|>0.5+15\Delta t^2,\ \ \left\|\Delta\mathbf{v}\right\|>3+30\Delta t', ...
 'N:violation or stale telemetry (position > 1 s)  ->  FAILSAFE  ->  PX4 NAV_LAND.', ...
 'H:Controller acceptance of an avoidance proposal', ...
 '\mathrm{age}\le0.5\,\mathrm{s},\ \ \left\|\mathbf{p}_{sp}-\mathbf{p}\right\|\le6\,\mathrm{m},\ \ h_{sp}\in[1.5,\,15]\,\mathrm{m}', ...
 'H:PID (inside PX4, background)', ...
 'u=K_Pe+K_I\int e\,dt+K_D\dot{e}', ...
 'H:Say clearly', ...
 'N:1) The application does not implement PID; PX4 does.  2) The response model is a FIT to|recorded PX4 flights, not a physical derivation.  3) All of this is simulation; nothing has|flown on hardware.  4) Avoidance needs about 0.9 s of warning; below ~0.7 s it failed every time.' ...
 }, 11);
savefig_if(f, exportFigures, outDir, 'sec11_cheatsheet');

fprintf('\nDone. %d figures. Set exportFigures = true to save PNGs to %s\n', numel(findall(0, 'Type', 'figure')), outDir);

%% ======================= LOCAL FUNCTIONS =================================
function f = newfig(name)
    f = figure('Name', name, 'Color', 'w', 'NumberTitle', 'off', 'Position', [40 40 1500 880]);
    annotation(f, 'textbox', [0 0.965 1 0.035], 'String', name, 'EdgeColor', 'none', 'FontSize', 14, ...
        'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Interpreter', 'none', 'Color', [0.1 0.3 0.6]);
end

function eqpanel(pos, items, fs)
% Stack LaTeX equations / notes top to bottom in one invisible axes.
%   'H:text'  bold heading    'N:text'  plain note ('|' = line break)    otherwise: LaTeX math (no $)
    ax = axes('Position', pos); axis(ax, 'off'); xlim(ax, [0 1]); ylim(ax, [0 1]); y = 0.995;
    for k = 1:numel(items)
        s = items{k};
        if strncmp(s, 'H:', 2)
            y = y - 0.012;
            h = text(ax, 0, y, s(3:end), 'Interpreter', 'none', 'FontWeight', 'bold', 'FontSize', fs + 1, 'VerticalAlignment', 'top', 'Color', [0.1 0.3 0.6]);
            y = y - h.Extent(4) - 0.012;
        elseif strncmp(s, 'N:', 2)
            h = text(ax, 0, y, strrep(s(3:end), '|', newline), 'Interpreter', 'none', 'FontSize', fs - 0.5, 'VerticalAlignment', 'top', 'Color', [0.25 0.25 0.25]);
            y = y - h.Extent(4) - 0.014;
        else
            h = text(ax, 0.01, y, ['$' s '$'], 'Interpreter', 'latex', 'FontSize', fs + 1.5, 'VerticalAlignment', 'top');
            y = y - h.Extent(4) - 0.014;
        end
    end
end

function savefig_if(f, doExport, outDir, name)
    drawnow;
    if doExport, exportgraphics(f, fullfile(outDir, [name '.png']), 'Resolution', 150); end
end

function [t, pos, vel, acc] = respSim(p0, v0, target, prm, a0)
% Exact port of simulate_response() in debris_avoidance.py for ONE setpoint (NED row vectors).
    dt = prm.dt; steps = round(prm.horizon/dt);
    p = p0(:)'; v = v0(:)'; a = a0(:)';
    gain = [min(1, dt/prm.tau_xy) min(1, dt/prm.tau_xy) min(1, dt/prm.tau_z)];
    pos = zeros(steps+1, 3); vel = pos; acc = pos; pos(1,:) = p; vel(1,:) = v; acc(1,:) = a;
    for k = 1:steps
        acmd = accelCommand(target(:)', p, v, prm);
        a = a + (acmd - a).*gain;
        p = p + v*dt + 0.5*a*dt^2;
        v = v + a*dt;
        pos(k+1,:) = p; vel(k+1,:) = v; acc(k+1,:) = a;
    end
    t = (0:steps)'*dt;
end

function a = accelCommand(target, p, v, prm)
% Port of _accel_command(): position error -> velocity command -> acceleration command (clamped).
    err = target - p;
    vxy = prm.kp_xy*err(1:2); sp = norm(vxy);
    vcmd = zeros(1, 3);
    vcmd(1:2) = vxy*min(1, prm.v_xy_max/max(sp, 1e-9));
    vcmd(3) = min(max(prm.kp_z*err(3), -prm.v_up_max), prm.v_down_max);
    a = zeros(1, 3);
    a(1:2) = min(max(prm.kv_xy*(vcmd(1:2) - v(1:2)), -prm.a_xy_max), prm.a_xy_max);
    a(3) = min(max(prm.kv_z*(vcmd(3) - v(3)), -prm.a_z_max), prm.a_z_max);
end

function [x, y] = limitSetpoint(position, target, speedLimit, pxP)
% Port of avoidance_interface.limit_setpoint().
    x = target(1); y = target(2);
    if speedLimit <= 0, return; end
    dx = x - position(1); dy = y - position(2); dist = hypot(dx, dy); reach = speedLimit/pxP;
    if dist <= reach, return; end
    x = position(1) + dx*reach/dist; y = position(2) + dy*reach/dist;
end

function s = ifelse(c, a, b)
    if c, s = a; else, s = b; end
end
