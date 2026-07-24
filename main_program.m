%% USV 4-DOF: Weighted A*, ALOS, dan AVO
clear; clc; close all;

global Rexp_log Rclear_log

Rexp_log = [];
Rclear_log = [];

global scale_log

scale_log = [];

% ===== DIAGNOSTIC LOGS (saturasi & clipping AVO/VO) =====
global ratio_log alpha_deg_log turn_req_deg_log turn_clipped_log

ratio_log        = [];
alpha_deg_log     = [];
turn_req_deg_log  = [];
turn_clipped_log  = [];
%% ===== PARAMETER ARENA & GRID =====
MAP_W    = 50;
MAP_H    = 33;
cellSize = 0.5;
Lship    = 1.6;

NX = ceil(MAP_W / cellSize);
NY = ceil(MAP_H / cellSize);
mapSize = [NY NX];
map = zeros(mapSize);

%% ===== Obstacles STATIS =====
obstacles = [20.0 20.0 0.25;
             40.0 20.0 0.25;
             10.0 10.0 0.25;
             30.0 10.0 0.25;
             17.0 16.5 0.25;
             41.0 16.0 0.25];

margin = 0.4;

[xg_m, yg_m] = meshgrid((0.5:1:NX-0.5)*cellSize, ...
                         (0.5:1:NY-0.5)*cellSize);

for k = 1:size(obstacles,1)
    cx = obstacles(k,1);
    cy = obstacles(k,2);
    r  = obstacles(k,3) + margin;
    map((xg_m-cx).^2 + (yg_m-cy).^2 <= r^2) = 1;
end

%% ===== Start, Waypoint, Goal =====
start    = [ 1.0  8.0];
waypoint = [25.0 20.0];
goal     = [48.0 13.0];

%% ===== PARAMETER AMAN =====
safeDist     = 0.5;
safeDistPlan = 1.5;

%% ===== Weighted A* =====
epsi = 1.5;

tic
pathVia = weightedAstar_grid_via( ...
    map,start,waypoint,goal, ...
    epsi,safeDistPlan,cellSize);
tPlan = toc;

fprintf('Planning time = %.6f s\n', tPlan);

if isempty(pathVia)
    error('Path not found S->WP->G (via WP).');
end

Lpath = sum(vecnorm(diff(pathVia),2,2));
fprintf('Raw path length = %.3f m\n', Lpath);
%% ===== Support point =====
%pathVia_wp = insert_support_points_around_wp(pathVia, waypoint, 0.10);
pathVia_wp = pathVia;

%% ===== G²-CBS PATH SMOOTHING:=====
tic

samplesPerSeg = 120; % Resolusi sampling kurva Bézier

epsRDP = 1.2; % RDP hanya menyisakan titik belok penting dari jalur grid A*

maxCurv = 0.45; % Nilai lebih kecil = belokan lebih lebar/halus.

pFinal = smooth_path_g2cbs_c2(pathVia_wp, samplesPerSeg, epsRDP, maxCurv);

minClrFinal = min_clearance_poly(pFinal, obstacles, safeDistPlan);

fprintf('Waktu G²-CBS = %.6f s\n', toc);
fprintf('Min clearance G²-CBS final = %.4f m\n', minClrFinal);

fprintf('\n=== Clearance per obstacle ===\n');
for k = 1:size(obstacles,1)
    c = obstacles(k,1:2);
    R = obstacles(k,3) + safeDistPlan;

    d = hypot(pFinal(:,1)-c(1), pFinal(:,2)-c(2)) - R;

    fprintf('Obs %d : %.4f m\n', k, min(d));
end

% Hanya untuk evaluasi waypoint
dmin_wp = min(vecnorm(pFinal - waypoint, 2, 2));
fprintf('Jarak minimum pFinal ke WP = %.4f m\n', dmin_wp);

Lfinal = sum(vecnorm(diff(pFinal),2,2));
fprintf('Final path length = %.3f m\n', Lfinal);

pSmooth0 = pFinal;
pGuard   = pFinal;

%% ===== Arc-length =====
cumM = cumulativeArc(pFinal);

%% ===== DYNAMIC OBSTACLE =====
useDynamicObs = true;
v_nom_dyn = 2.0;
totalLen  = cumM(end);

[~, idx_c1] = min(abs(pFinal(:,1) - 15));
Pc1 = pFinal(idx_c1,:); s1 = cumM(idx_c1); t_cross1 = s1 / v_nom_dyn;
[~, idx_c2] = min(abs(pFinal(:,1) - 37));
Pc2 = pFinal(idx_c2,:); s2 = cumM(idx_c2); t_cross2 = s2 / v_nom_dyn;

dynR = 0.25;
dyn1.x  = 15.0; dyn1.y0 = 8.0;  dyn1.v = 0.53; dyn1.R = dynR;
dyn2.x  = 37.0; dyn2.y0 = 22.0; dyn2.v = 0.18;  dyn2.R = dynR;

fprintf('Dynamic obs 1: x=%.2f, y0=%.2f, v=%.3f m/s (target t=%.2f s)\n', ...
    dyn1.x, dyn1.y0, dyn1.v, t_cross1);
fprintf('Dynamic obs 2: x=%.2f, y0=%.2f, v=%.3f m/s (target t=%.2f s)\n', ...
    dyn2.x, dyn2.y0, dyn2.v, t_cross2);

%% ===== Figure Dynamic Obstacles Only =====
figure(99); clf;
set(gcf,'Name','Dynamic Obstacles Only',...
         'NumberTitle','off',...
         'Color','w');

hold on;
axis equal;
grid on;

xlabel('X [m]');
ylabel('Y [m]');
title('Dynamic Obstacles Only');

axis([0 MAP_W 0 MAP_H]);

theta99 = linspace(0,2*pi,80);

x_d1_0 = dyn1.x;
y_d1_0 = dyn1.y0;

x_d2_0 = dyn2.x;
y_d2_0 = dyn2.y0;

%% Titik awal obstacle
plot(x_d1_0,y_d1_0,...
    'o',...
    'MarkerFaceColor',[0.85 0.45 0.10],...
    'MarkerEdgeColor','k',...
    'MarkerSize',8,...
    'DisplayName','Dyn Obs Start');

plot(x_d2_0,y_d2_0,...
    'o',...
    'MarkerFaceColor',[0.85 0.45 0.10],...
    'MarkerEdgeColor','k',...
    'MarkerSize',8,...
    'HandleVisibility','off');

%% Obstacle bergerak
hDyn1_99 = plot(x_d1_0,y_d1_0,'o',...
    'Color',[1 0.5 0],...
    'MarkerFaceColor',[1 0.8 0],...
    'MarkerSize',8,...
    'DisplayName','Dyn Obs 1');

hDyn2_99 = plot(x_d2_0,y_d2_0,'o',...
    'Color',[1 0.3 0],...
    'MarkerFaceColor',[1 0.6 0],...
    'MarkerSize',8,...
    'DisplayName','Dyn Obs 2');

%% Safety circle
hDyn1Ring_99 = plot( ...
    x_d1_0+(dynR+safeDist)*cos(theta99), ...
    y_d1_0+(dynR+safeDist)*sin(theta99), ...
    '--','Color',[1 0.6 0],...
    'LineWidth',0.8,...
    'HandleVisibility','off');

hDyn2Ring_99 = plot( ...
    x_d2_0+(dynR+safeDist)*cos(theta99), ...
    y_d2_0+(dynR+safeDist)*sin(theta99), ...
    '--','Color',[1 0.4 0],...
    'LineWidth',0.8,...
    'HandleVisibility','off');

%% Trail
dyn1_traj99 = [x_d1_0 y_d1_0];
dyn2_traj99 = [x_d2_0 y_d2_0];

hDynTrail1_99 = plot(NaN,NaN,'--',...
    'Color',[1 0.5 0],...
    'LineWidth',1.5,...
    'DisplayName','Trail Obs 1');

hDynTrail2_99 = plot(NaN,NaN,'--',...
    'Color',[1 0.3 0],...
    'LineWidth',1.5,...
    'DisplayName','Trail Obs 2');

legend('Location','best');

%% ===== Figure 10 : Environment Only =====
figure(10); clf;
set(gcf,'Name','Environment Only',...
         'NumberTitle','off',...
         'Color','w');

hold on;
axis equal;
grid on;

xlabel('X [m]');
ylabel('Y [m]');
title('Environment');

axis([0 MAP_W 0 MAP_H]);

theta10 = linspace(0,2*pi,80);


%% ===== Static Obstacles =====
for k = 1:size(obstacles,1)

    if k == 1
        fill(obstacles(k,1)+obstacles(k,3)*cos(theta10), ...
             obstacles(k,2)+obstacles(k,3)*sin(theta10), ...
             'r','FaceAlpha',0.25,...
             'EdgeColor','none',...
             'DisplayName','Static Obs');
    else
        fill(obstacles(k,1)+obstacles(k,3)*cos(theta10), ...
             obstacles(k,2)+obstacles(k,3)*sin(theta10), ...
             'r','FaceAlpha',0.25,...
             'EdgeColor','none',...
             'HandleVisibility','off');
    end

    plot(obstacles(k,1)+(obstacles(k,3)+safeDist)*cos(theta10), ...
         obstacles(k,2)+(obstacles(k,3)+safeDist)*sin(theta10), ...
         'r--','LineWidth',0.8,...
         'HandleVisibility','off');

end

%% ===== Start - WP - Goal =====
plot(start(1),start(2),...
    'yo','MarkerFaceColor','y',...
    'DisplayName','Start');

plot(waypoint(1),waypoint(2),...
    'mo','MarkerFaceColor','m',...
    'DisplayName','Waypoint');

plot(goal(1),goal(2),...
    'ro','MarkerFaceColor','r',...
    'DisplayName','Goal');

%% ===== Dynamic Obstacles =====
if useDynamicObs
x_d1_0 = dyn1.x;
y_d1_0 = dyn1.y0;

x_d2_0 = dyn2.x;
y_d2_0 = dyn2.y0;

% titik awal obstacle
plot(x_d1_0,y_d1_0,...
    'o',...
    'MarkerFaceColor',[0.85 0.45 0.10],...
    'MarkerEdgeColor','k',...
    'MarkerSize',8,...
    'DisplayName','Dyn Obs Start');

plot(x_d2_0,y_d2_0,...
    'o',...
    'MarkerFaceColor',[0.85 0.45 0.10],...
    'MarkerEdgeColor','k',...
    'MarkerSize',8,...
    'HandleVisibility','off');

% obstacle bergerak
hDyn1_env = plot(x_d1_0,y_d1_0,'o',...
    'Color',[1 0.5 0],...
    'MarkerFaceColor',[1 0.8 0],...
    'MarkerSize',8,...
    'DisplayName','Dyn Obs 1');

hDyn2_env = plot(x_d2_0,y_d2_0,'o',...
    'Color',[1 0.3 0],...
    'MarkerFaceColor',[1 0.6 0],...
    'MarkerSize',8,...
    'DisplayName','Dyn Obs 2');

% safety circle
hDyn1Ring_env = plot( ...
    x_d1_0+(dynR+safeDist)*cos(theta10), ...
    y_d1_0+(dynR+safeDist)*sin(theta10), ...
    '--','Color',[1 0.6 0],...
    'LineWidth',0.8,...
    'HandleVisibility','off');

hDyn2Ring_env = plot( ...
    x_d2_0+(dynR+safeDist)*cos(theta10), ...
    y_d2_0+(dynR+safeDist)*sin(theta10), ...
    '--','Color',[1 0.4 0],...
    'LineWidth',0.8,...
    'HandleVisibility','off');

% trail
dyn1_traj_env = [x_d1_0 y_d1_0];
dyn2_traj_env = [x_d2_0 y_d2_0];

hDynTrail1_env = plot(NaN,NaN,'--',...
    'Color',[1 0.5 0],...
    'LineWidth',1.5,...
    'DisplayName','Trail Obs 1');

hDynTrail2_env = plot(NaN,NaN,'--',...
    'Color',[1 0.3 0],...
    'LineWidth',1.5,...
    'DisplayName','Trail Obs 2');

legend('Location','northeastoutside');

end
%% ===== Figure 1 =====
figure(1); clf;
set(gcf, 'Name','Figure 1', 'NumberTitle','off', 'Color','w');
hold on; axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]');
axis([0 MAP_W 0 MAP_H]);
title('Weighted A* Global Plan');

theta = linspace(0, 2*pi, 80);
for k = 1:size(obstacles,1)
    if k == 1
        fill(obstacles(k,1)+obstacles(k,3)*cos(theta), ...
             obstacles(k,2)+obstacles(k,3)*sin(theta), ...
             'r','FaceAlpha',0.25,'EdgeColor','none','DisplayName','Static Obs');
    else
        fill(obstacles(k,1)+obstacles(k,3)*cos(theta), ...
             obstacles(k,2)+obstacles(k,3)*sin(theta), ...
             'r','FaceAlpha',0.25,'EdgeColor','none','HandleVisibility','off');
    end
    plot(obstacles(k,1)+(obstacles(k,3)+safeDist)*cos(theta), ...
         obstacles(k,2)+(obstacles(k,3)+safeDist)*sin(theta), ...
         'r--','LineWidth',0.8,'HandleVisibility','off');
end
plot(start(1),start(2),'yo','MarkerFaceColor','y','DisplayName','Start');
plot(waypoint(1),waypoint(2),'mo','MarkerFaceColor','m','DisplayName','Waypoint');
plot(goal(1),goal(2),'ro','MarkerFaceColor','r','DisplayName','Goal');
plot(pathVia(:,1), pathVia(:,2), 'm-.','LineWidth',1.2,'DisplayName','Weighted A* Path');
plot(pFinal(:,1),  pFinal(:,2),  'k-', 'LineWidth',2.2,'DisplayName','C² Continuous Smoothed Path');
legend('Location','northeastoutside');

%% ===== PARAMETER DINAMIKA 4-DOF & CONTROLLER =====
params = struct();
% Surge
params.A1=1.5066; params.A2=-0.7405; params.A3=0.4219; params.A4=-0.1397; params.A18=0.0178;
% Sway
params.A5=-0.1464; params.A6=-3.1952; params.A7=4.1189; params.A8=0.0; params.A9=0.0;
% Yaw
params.A10=0.0845; params.A11=0.0561; params.A12=-1.0495; params.A13=1.4038;
params.A14=-2.0764; params.A15=0.0010; params.A16=0.9671; params.A17=0.0021;
params.A19=0.0010; params.A20=0.0; params.A21=0.0; params.A22=0.0;
% Roll
params.KpLin=0.0; params.KpAbs=0.0; params.KpCub=0.0; params.Kphi=13.5523;
params.Kfy=-0.0175; params.Kv=-3.3096; params.Kr=-2.7576; params.Kdelta=0.1738; params.Kbias=-0.3631;
params.g = 9.81;

lims.TX=200; lims.TY=60; lims.TN=80; lims.TK=60;
TK_ff_bias = -params.Kbias;

% PID GAINS
gains.Ku_p   = 50;    gains.Ku_i   = 2.0;   gains.Ku_d   = 4;
gains.Kpsi_p = 28;    gains.Kpsi_i = 2.5;   gains.Kpsi_d = 10;
gains.Kphi_p = 12;     gains.Kphi_i = 0.05;   gains.Kphi_d = 12;


dt    = 0.05;
u_ref = 1.5;
animPause = 0.02;

align.e_align_deg = 20;
align.Kpsi_align  = 1.0;
align.r_max_cmd   = 0.50;

guard.buffer       = 1.4;
guard.yaw_gain     = 1.2;
guard.target_shift = 1.0;
guard.minLdScale   = 0.45;
guard.slowdownMin  = 0.15;

% ALOS
alos.Delta = 1.0;
alos.gamma = 0.25;

% Banking
bank.k_bank      = 0.8;
bank.phi_max_deg = 5;
bank.phi_max     = deg2rad(bank.phi_max_deg);

%% ===== PARAMETER DISTURBANCE ANGIN =====
wind.use      = false;            % saklar aktif/nonaktif disturbance angin
wind.V_tw     = 3.0;             % kecepatan angin [m/s] (ubah utk uji sensitivitas)
wind.beta_tw  = deg2rad(0);      % arah datang angin, sumbu bumi/NED [rad] (0=dr Timur, CCW)
wind.rho_a    = 1.225;           % massa jenis udara [kg/m^3] (kondisi permukaan laut standar)
wind.A_Fw     = 0.08;            
wind.A_Lw     = 0.30;            
wind.Cdx      = 0.70;            
wind.Cdy      = 0.90;            
wind.Loa      = Lship;           
wind.kU       = params.A18;
wind.kV       = params.A18;

% Gambar indikator arah angin pada figure peta yg sudah dibuat sebelumnya
if wind.use
    figure(10); draw_wind_indicator(wind.beta_tw, wind.V_tw, MAP_W, MAP_H);
    figure(1);  draw_wind_indicator(wind.beta_tw, wind.V_tw, MAP_W, MAP_H);
end

%% ===== PARAMETER AVO =====
% SAKLAR MODE: true  -> AVO (Adaptive Velocity Obstacle, radius cone
%                        membesar sesuai closing speed terhadap obstacle)
%              false -> VO klasik (radius cone TETAP / tidak adaptif,
%                        definisi Velocity Obstacle standar)
useAdaptiveVO = true;

if useAdaptiveVO
    algoLabel = 'AVO';
else
    algoLabel = 'VO';
end

useLocalOA = true;      % true = AVO/VO aktif
                        % false = tanpa local obstacle avoidance


avo.gamma         = 1.2 * useAdaptiveVO;    
avo.lambda        = 0.35;                   
avo.max_expand_ratio = 1.22;                 
avo.detect_R      = 8.0;   % hanya untuk mendeteksi / menghitung risiko
avo.activate_R    = 4.5;   % baru boleh MASUK AVO jika obstacle sudah dekat
avo.release_R     = 6.0;   % baru boleh KELUAR AVO jika obstacle menjauh
avo.safe_dist     = 0.5;   % Radius switching ALOS - AVO
avo.detect_R   = 8.0;   % hanya mendeteksi / memilih obstacle kandidat
avo.activate_R = 4.5;   % baru masuk AVO jika obstacle cukup dekat
avo.release_R  = 6.0;   % baru kembali ALOS jika obstacle sudah menjauh
avo.margin        = 0.1;
avo.R_robot       = Lship/2;
avo.min_speed     = 0.45;
avo.max_speed     = u_ref;
avo.soft_alpha    = 0.30;
avo.bias_boundary = 0.08;
avo.R_clear = avo.R_robot + dynR + avo.safe_dist;

%% ===== PARAMETER BLENDING & HYSTERESIS =====
blend.d_enter        = 3.5;
blend.d_exit         = 4.0;
blend.dcpa_critical  = 0.4;
blend.dcpa_safe      = avo.R_clear;
blend.tcpa_max       = 12.0;  % hanya untuk pemilihan obstacle kandidat
blend.tau_blend      = 0.6;
blend.tau_release    = 1.2;
blend.tcpa_enter     = 7.0;  % masuk AVO jika potensi tabrakan <= 4 detik
blend.dwell_min      = 2.5;  % tahan AVO minimal 2.5 detik
blend.cte_recover_th = 0.5;

%% ===== VISUALISASI 2D =====
figure(2); clf;
set(gcf, 'Name','Figure 2', 'NumberTitle','off', 'Color','w');
hold on; axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]');
axis([0 MAP_W 0 MAP_H]);
title(sprintf('USV Dynamic 4-DOF (Weighted A* dan %s)', algoLabel));

hHUD = annotation(gcf,'textbox', ...
    [0.03 0.70 0.18 0.18], ...
    'String','', ...
    'FitBoxToText','on', ...
    'BackgroundColor',[0.97 0.97 0.97], ...
    'EdgeColor','k', ...
    'LineWidth',1.2, ...
    'FontSize',9, ...
    'FontWeight','bold', ...
    'Interpreter','none', ...
    'HorizontalAlignment','left', ...
    'VerticalAlignment','middle');
for k = 1:size(obstacles,1)
    if k == 1
        fill(obstacles(k,1)+obstacles(k,3)*cos(theta), ...
             obstacles(k,2)+obstacles(k,3)*sin(theta), ...
             'r','FaceAlpha',0.25,'EdgeColor','none','DisplayName','Static Obs');
    else
        fill(obstacles(k,1)+obstacles(k,3)*cos(theta), ...
             obstacles(k,2)+obstacles(k,3)*sin(theta), ...
             'r','FaceAlpha',0.25,'EdgeColor','none','HandleVisibility','off');
    end
    plot(obstacles(k,1)+(obstacles(k,3)+safeDist)*cos(theta), ...
         obstacles(k,2)+(obstacles(k,3)+safeDist)*sin(theta), ...
         'r--','LineWidth',0.8,'HandleVisibility','off');
end

plot(start(1),start(2),'yo','MarkerFaceColor','y','DisplayName','Start');
plot(waypoint(1),waypoint(2),'mo','MarkerFaceColor','m','DisplayName','Waypoint');
plot(goal(1),goal(2),'ro','MarkerFaceColor','r','DisplayName','Goal');
plot(pFinal(:,1),pFinal(:,2),'k-','LineWidth',1.0,'DisplayName','Final Smooth Weighted A*');


%% ===== Dynamic Obstacles (Triangle Style) =====
if useDynamicObs
x_d1_0 = dyn1.x;
y_d1_0 = dyn1.y0;

x_d2_0 = dyn2.x;
y_d2_0 = dyn2.y0;


% titik awal obstacle
plot(x_d1_0,y_d1_0,...
    'o',...
    'MarkerFaceColor',[0.85 0.45 0.10],...
    'MarkerEdgeColor','k',...
    'MarkerSize',8,...
    'DisplayName','Dyn Obs Start');

plot(x_d2_0,y_d2_0,...
    'o',...
    'MarkerFaceColor',[0.85 0.45 0.10],...
    'MarkerEdgeColor','k',...
    'MarkerSize',8,...
    'HandleVisibility','off');

% garis lintasan obstacle
hDynPath1 = plot(NaN,NaN,...
    '--',...
    'Color',[0.85 0.65 0.35],...
    'LineWidth',1.3,...
    'DisplayName','Dyn Obs Path');

hDynPath2 = plot(NaN,NaN,...
    '--',...
    'Color',[0.85 0.65 0.35],...
    'LineWidth',1.3,...
    'HandleVisibility','off');

% obstacle dinamis = lingkaran
x_d1_0 = dyn1.x; y_d1_0 = dyn1.y0;
x_d2_0 = dyn2.x; y_d2_0 = dyn2.y0;

hDyn1 = plot(x_d1_0, y_d1_0, 'o', ...
    'Color',[1 0.5 0],...
    'MarkerFaceColor',[1 0.8 0],...
    'MarkerSize',8,...
    'DisplayName','Dyn obs 1');

hDyn2 = plot(x_d2_0, y_d2_0, 'o', ...
    'Color',[1 0.3 0],...
    'MarkerFaceColor',[1 0.6 0],...
    'MarkerSize',8,...
    'DisplayName','Dyn obs 2');

hDyn1Ring = plot( ...
    x_d1_0+(dynR+safeDist)*cos(theta), ...
    y_d1_0+(dynR+safeDist)*sin(theta), ...
    'Color',[1 0.6 0],...
    'LineStyle','--',...
    'LineWidth',0.8,...
    'HandleVisibility','off');

hDyn2Ring = plot( ...
    x_d2_0+(dynR+safeDist)*cos(theta), ...
    y_d2_0+(dynR+safeDist)*sin(theta), ...
    'Color',[1 0.4 0],...
    'LineStyle','--',...
    'LineWidth',0.8,...
    'HandleVisibility','off');

dyn1_traj = [x_d1_0 y_d1_0];
dyn2_traj = [x_d2_0 y_d2_0];

hDynTrail1 = plot(NaN,NaN,'--',...
    'Color',[1 0.5 0],...
    'LineWidth',1.5,...
    'DisplayName','Trail Dyn Obs');

hDynTrail2 = plot(NaN,NaN,'--',...
    'Color',[1 0.3 0],...
    'LineWidth',1.5,...
    'HandleVisibility','off');
end
hTraj = plot(NaN,NaN,'g-','LineWidth',1.8,'DisplayName','Trajectory');

shipScale = 1.0;
tKapal = hgtransform('Parent', gca);
baseShip = shipScale * [1.00 0.00; -0.80 0.45; -0.40 0.00; -0.80 -0.45];
patch('XData',baseShip(:,1),'YData',baseShip(:,2), ...
      'FaceColor',[0 0.7 0],'EdgeColor','k','LineWidth',0.8, ...
      'Parent',tKapal,'HandleVisibility','off');
legend('Location','northeastoutside');

if wind.use
    draw_wind_indicator(wind.beta_tw, wind.V_tw, MAP_W, MAP_H);
end

%% ===== State awal & logs =====
x   = start(1);
y   = start(2);
psi0 = atan2(waypoint(2)-start(2), ...
             waypoint(1)-start(1));
psi = psi0;
phi = 0;
Vb  = [0;0;0;0];

set(tKapal,'Matrix',makehgtform('translate',[x y 0],'zrotate',psi));

traj=[];  dist_log=[];  time_log=[];  cte_log=[];  t=0;
x_log=[];  y_log=[];  psi_log=[];  phi_log=[];
x_des_log=[];  y_des_log=[];  psi_des_log=[];  phi_des_log=[];
beta_hat_log = [];
u_log=[];  v_log=[];  p_log=[];  r_log=[];  speed_log=[];
w_avo_log = []; 
Xwind_log = []; Ywind_log = [];

goal_tol = 1.0;

eInt_u   = 0;  eInt_psi = 0;  eInt_phi = 0;
intMax_u = 40; intMax_psi = 10; intMax_phi = 5;

[~, idx_wp_on_p] = min(vecnorm(pFinal - waypoint, 2, 2));
s_wp    = cumM(idx_wp_on_p);
wp_zone = 3.0;

%% ===== STATE BLENDING ALOS <-> AVO =====
w_avo        = 0;
in_avo_state = false;
t_enter_avo  = -inf;
avo_side_locked = 0;   % 0 = belum dikunci, +1/-1 = sisi hindar yang sudah dipilih

disp('=== Tracking Single Path 4-DOF (ALOS <-> AVO smooth blending) ===');

% seed awal log
dObs0 = inf;
for k = 1:size(obstacles,1)
    cx = obstacles(k,1); cy = obstacles(k,2); r0 = obstacles(k,3);
    dObs0 = min(dObs0,...
    hypot(x-cx,y-cy) ...
    - r0 ...
    - avo.R_robot);
    
end
time_log = [time_log; 0];
dist_log = [dist_log; dObs0];
[~, e_ct0] = cte_by_projection(pFinal, [x y]);
cte_log     = [cte_log;  e_ct0];
x_log       = [x_log;   x];
y_log       = [y_log;   y];
psi_log     = [psi_log; psi];
phi_log     = [phi_log; phi];
x_des_log   = [x_des_log; x];
y_des_log   = [y_des_log; y];
psi_des_log = [psi_des_log; psi];
phi_des_log = [phi_des_log; 0];
beta_hat    = 0;
beta_hat_log= [beta_hat_log; beta_hat];
u_log       = [u_log;   Vb(1)];
v_log       = [v_log;   Vb(2)];
p_log       = [p_log;   Vb(3)];
r_log       = [r_log;   Vb(4)];
speed_log   = [speed_log; hypot(Vb(1),Vb(2))];
w_avo_log   = [w_avo_log; 0];
Xwind_log   = [Xwind_log; 0];
Ywind_log   = [Ywind_log; 0];

psi_des_prev = psi;
phi_des_prev = 0;
last_idx     = 1;
tMax         = 120;

%% ===== MAIN LOOP =====
while true
     d_goal = hypot(goal(1)-x, goal(2)-y);
    if d_goal < goal_tol
        fprintf('Goal reached (d = %.3f m)\n', d_goal);
        break;
    end
    if t >= tMax
        warning('Simulation timeout at t=%.1f s', t);
        break;
    end

    % ===== Posisi obstacle DINAMIS =====
    if useDynamicObs
    x_d1 = dyn1.x;
    y_d1 = dyn1.y0 + dyn1.v*t;

    x_d2 = dyn2.x;
    y_d2 = dyn2.y0 - dyn2.v*t;

    y_d1 = min(max(y_d1,0),MAP_H);
    y_d2 = min(max(y_d2,0),MAP_H);

    set(hDyn1,'XData',x_d1,'YData',y_d1);
    set(hDyn2,'XData',x_d2,'YData',y_d2);

else
    x_d1 = -1000;
    y_d1 = -1000;

    x_d2 = -1000;
    y_d2 = -1000;
end
   
%% ===== Update Figure 10 =====

if useDynamicObs

    set(hDyn1_env,'XData',x_d1,'YData',y_d1);
    set(hDyn2_env,'XData',x_d2,'YData',y_d2);

    dyn1_traj_env = [dyn1_traj_env; x_d1 y_d1];
    dyn2_traj_env = [dyn2_traj_env; x_d2 y_d2];

    set(hDynTrail1_env,...
        'XData',dyn1_traj_env(:,1),...
        'YData',dyn1_traj_env(:,2));

    set(hDynTrail2_env,...
        'XData',dyn2_traj_env(:,1),...
        'YData',dyn2_traj_env(:,2));

    set(hDyn1Ring_env,...
        'XData',x_d1+(dynR+safeDist)*cos(theta10),...
        'YData',y_d1+(dynR+safeDist)*sin(theta10));

    set(hDyn2Ring_env,...
        'XData',x_d2+(dynR+safeDist)*cos(theta10),...
        'YData',y_d2+(dynR+safeDist)*sin(theta10));

end
%% ===== Update Figure 99 =====

if isgraphics(hDyn1_99)

    set(hDyn1_99,'XData',x_d1,'YData',y_d1);
    set(hDyn2_99,'XData',x_d2,'YData',y_d2);

    dyn1_traj99 = [dyn1_traj99; x_d1 y_d1];
    dyn2_traj99 = [dyn2_traj99; x_d2 y_d2];

    set(hDynTrail1_99,...
        'XData',dyn1_traj99(:,1),...
        'YData',dyn1_traj99(:,2));

    set(hDynTrail2_99,...
        'XData',dyn2_traj99(:,1),...
        'YData',dyn2_traj99(:,2));

    set(hDyn1Ring_99,...
        'XData',x_d1+(dynR+safeDist)*cos(theta99),...
        'YData',y_d1+(dynR+safeDist)*sin(theta99));

    set(hDyn2Ring_99,...
        'XData',x_d2+(dynR+safeDist)*cos(theta99),...
        'YData',y_d2+(dynR+safeDist)*sin(theta99));

end

    if useDynamicObs

    dyn1_traj = [dyn1_traj; x_d1 y_d1];
    dyn2_traj = [dyn2_traj; x_d2 y_d2];

    set(hDynTrail1,'XData',dyn1_traj(:,1),'YData',dyn1_traj(:,2));
    set(hDynTrail2,'XData',dyn2_traj(:,1),'YData',dyn2_traj(:,2));

    set(hDyn1Ring,...
        'XData',x_d1+(dynR+safeDist)*cos(theta),...
        'YData',y_d1+(dynR+safeDist)*sin(theta));

    set(hDyn2Ring,...
        'XData',x_d2+(dynR+safeDist)*cos(theta),...
        'YData',y_d2+(dynR+safeDist)*sin(theta));

end

    obs_all = [obstacles; x_d1, y_d1, dyn1.R; x_d2, y_d2, dyn2.R];

    % ---- ALOS ----
    [s_on, remaining, projPoint, psi_id, y_e, last_idx] = ...
        projectArc_alos(pFinal, cumM, [x y], last_idx);
    x_ref = projPoint(1);
    y_ref = projPoint(2);
    dist_s_to_wp = s_wp - s_on;
    dist_wp_abs  = abs(dist_s_to_wp);

    u = Vb(1);  v = Vb(2);  p = Vb(3);  r = Vb(4);
    U = hypot(u, v);

    Delta_eff = max(0.6, alos.Delta + 0.2*U);
    if dist_wp_abs < wp_zone
        Delta_eff = 1.2 + 0.5*(dist_wp_abs/wp_zone);
    end

    denom = Delta_eff^2 + (y_e + Delta_eff*beta_hat)^2;
    if denom < 1e-6, denom = 1e-6; end
    beta_hat_dot = alos.gamma * sqrt(U * Delta_eff / denom) * y_e;
    beta_hat     = beta_hat + beta_hat_dot*dt;

    k_cte = 1.2;
    psi_los = psi_id + atan2(-k_cte*y_e, Delta_eff);
    psi_correction = 0;
    max_corr       = deg2rad(20);
    psi_correction = max(-max_corr, min(max_corr, psi_correction));
    psi_base = wrapToPi(psi_los + psi_correction);

    % STATIC GUARD
    d_min_stat = inf;  avoid_dir = [0 0];  r0 = 0;
    for kk = 1:size(obstacles,1)
        cx = obstacles(kk,1);  cy = obstacles(kk,2);  rr = obstacles(kk,3);
        v_ob = [x-cx, y-cy];  d = norm(v_ob);
        if d < d_min_stat
            d_min_stat = d;
            if d > 1e-9, avoid_dir = v_ob/d; else, avoid_dir = [0 0]; end
            r0 = rr;
        end
    end
    clr_stat      = d_min_stat - r0;
    u_ref_eff     = u_ref;
    yaw_avoid_stat = 0;

    if clr_stat < guard.buffer*safeDist
        if norm(avoid_dir) > 0
            psi_away       = atan2(avoid_dir(2), avoid_dir(1));
            dpsiAway       = atan2(sin(psi_away-psi), cos(psi_away-psi));
            yaw_avoid_stat = guard.yaw_gain * dpsiAway;
        end
        if (clr_stat - safeDist) < 0
            scale     = max(guard.slowdownMin, clr_stat/safeDist);
            u_ref_eff = u_ref * max(0.05, min(1.0, scale));
        end
    end

    if dist_wp_abs < wp_zone, yaw_avoid_stat = 0; end

    if abs(y_e) > 0.6
        psi_base = wrapToPi(psi_los + 0.25*atan2(-y_e, 1.5));
    else
        psi_base = wrapToPi(psi_los + psi_correction + yaw_avoid_stat);
    end

    % ===== AVO setup =====
    pA  = [x; y];
    pB1 = [x_d1; y_d1];  vB1 = [0;  dyn1.v];
    pB2 = [x_d2; y_d2];  vB2 = [0; -dyn2.v];

    R2       = [cos(psi) -sin(psi); sin(psi) cos(psi)];
    vA_world = R2 * [u; v];

   % ===== Evaluasi risiko masing-masing dynamic obstacle =====
d1_cc = norm(pA - pB1);
d2_cc = norm(pA - pB2);

% --- Obstacle 1 ---
r_rel1 = pB1 - pA;
v_rel1 = vB1 - vA_world;

if norm(v_rel1)^2 < 1e-6
    tcpa1 = inf;
else
    tcpa1 = -dot(r_rel1, v_rel1) / (norm(v_rel1)^2 + 1e-6);
end

dcpa1 = norm(r_rel1 + max(0,tcpa1)*v_rel1);

% Obstacle 1 hanya dianggap kandidat jika sudah masuk detection range
if d1_cc < avo.detect_R && tcpa1 > 0 && tcpa1 < blend.tcpa_max
    risk1 = max(0, 1 - dcpa1/blend.dcpa_safe) * ...
            max(0, 1 - tcpa1/blend.tcpa_max);
else
    risk1 = 0;
end


% --- Obstacle 2 ---
r_rel2 = pB2 - pA;
v_rel2 = vB2 - vA_world;

if norm(v_rel2)^2 < 1e-6
    tcpa2 = inf;
else
    tcpa2 = -dot(r_rel2, v_rel2) / (norm(v_rel2)^2 + 1e-6);
end

dcpa2 = norm(r_rel2 + max(0,tcpa2)*v_rel2);

% Obstacle 2 hanya dianggap kandidat jika sudah masuk detection range
if d2_cc < avo.detect_R && tcpa2 > 0 && tcpa2 < blend.tcpa_max
    risk2 = max(0, 1 - dcpa2/blend.dcpa_safe) * ...
            max(0, 1 - tcpa2/blend.tcpa_max);
else
    risk2 = 0;
end

% ===== Pilih obstacle paling berisiko, bukan paling dekat =====
if risk1 >= risk2
    pB = pB1;
    vB = vB1;
    d_dyn = d1_cc;
    tcpa = tcpa1;
    dcpa = dcpa1;
    R_obs_sel = dyn1.R;
    obs_active_id = 1;
else
    pB = pB2;
    vB = vB2;
    d_dyn = d2_cc;
    tcpa = tcpa2;
    dcpa = dcpa2;
    R_obs_sel = dyn2.R;
    obs_active_id = 2;
end

if useLocalOA
    enter_avo = (d_dyn <= avo.activate_R) && ...
                (tcpa > 0) && ...
                (tcpa <= blend.tcpa_enter) && ...
                (dcpa < avo.R_clear);
else
    enter_avo = false;
end

exit_avo = (tcpa <= 0) || ...
           (d_dyn >= avo.release_R);

% State machine ALOS - AVO
if ~in_avo_state

    % Default: tetap ALOS
    if enter_avo
        in_avo_state = true;
        t_enter_avo  = t;
        avo_side_locked = 0;   % encounter baru -> boleh pilih sisi lagi
    end

else

    % Setelah AVO aktif, tahan minimal dwell_min agar tidak flickering
    if exit_avo && ((t - t_enter_avo) >= blend.dwell_min)
        in_avo_state = false;
        avo_side_locked = 0;
    end
end


% OUTPUT GUIDANCE: ALOS normal atau AVO
v_pref = u_ref_eff * [cos(psi_base); sin(psi_base)];

if in_avo_state

    if useAdaptiveVO
        [v_cmd_xy, avo_side_locked] = avo_select_velocity_xy( ...
            pA, vA_world, v_pref, ...
            pB, vB, ...
            R_obs_sel, avo.R_robot, ...
            avo, d_dyn, avo_side_locked);
    else
        v_cmd_xy = vo_select_velocity_xy( ...
            pA, vA_world, v_pref, ...
            pB, vB, ...
            R_obs_sel, avo.R_robot, ...
            avo.safe_dist);
    end

    psi_avo = atan2(v_cmd_xy(2), v_cmd_xy(1));
    dpsi_avo = wrapToPi(psi_avo - psi_base);

    % Batasi perubahan heading AVO agar tidak terlalu ekstrem
    max_avo_turn = deg2rad(50);

    turn_req_deg_log(end+1) = rad2deg(dpsi_avo);
    turn_clipped_log(end+1) = abs(dpsi_avo) > max_avo_turn;

    dpsi_avo = max(-max_avo_turn, min(max_avo_turn, dpsi_avo));

    psi_des_avo = wrapToPi(psi_base + dpsi_avo);
    u_cmd_nom   = norm(v_cmd_xy);

    % Hanya untuk log/plot:
    w_avo = 1;

else
    % Tidak ada collision risk:
    % ALOS dipakai penuh, AVO tidak ikut campur.
    psi_des_avo = psi_base;
    u_cmd_nom   = u_ref_eff;

    % Hanya untuk log/plot:
    w_avo = 0;
end

    % HUD
    if in_avo_state
    mode_str = algoLabel;   % "AVO" atau "VO"
else
    mode_str = "ALOS";
end
    clr = d_dyn - (R_obs_sel + avo.R_robot);
   hudStr = sprintf([ ...
    'MODE : %s\n' ...
    '| Time : %.1f s\n' ...
    '| w    : %.2f\n' ...
    '| d    : %.2f m\n' ...
    '| DCPA : %.2f m\n' ...
    '| TCPA : %.1f s\n' ...
    '| Clr  : %.2f m\n' ...
    '| U    : %.2f m/s'], ...
    mode_str, ...
    t, ...
    w_avo, ...
    d_dyn, ...
    dcpa, ...
    max(-99,min(99,tcpa)), ...
    clr, ...
    U);

set(hHUD,'String',hudStr);
    % ALIGN gate
    e_psi_for_align = atan2(sin(psi_des_avo-psi), cos(psi_des_avo-psi));
    if abs(rad2deg(e_psi_for_align)) > align.e_align_deg
        u_cmd = max(0.3, u_cmd_nom);
    else
        u_cmd = u_cmd_nom;
    end

    % low-pass psi_des
    alpha_psi = 0.20;
    dpsi      = atan2(sin(psi_des_avo - psi_des_prev), cos(psi_des_avo - psi_des_prev));
    psi_des   = wrapToPi(psi_des_prev + alpha_psi*dpsi);
    psi_des_prev = psi_des;

    % ===== PID CONTROLLER 4-DOF =====
    e_u   = u_cmd - u;
    e_psi = e_psi_for_align;

    Ld_bank = max(0.5, Delta_eff);
    kappa   = 2 * sin(e_psi) / Ld_bank;
    U_eff   = max(0.3, U);
    a_y_cmd = (U_eff^2) * kappa;
    phi_cmd = bank.k_bank * atan(a_y_cmd / params.g);
    phi_cmd = max(-bank.phi_max, min(bank.phi_max, phi_cmd));

    tau_phi   = 1.2;
    alpha_phi = dt / (tau_phi + dt);
    phi_des   = phi_des_prev + alpha_phi*(phi_cmd - phi_des_prev);
    phi_des_prev = phi_des;

    e_phi = phi_des - phi;

    eInt_u   = eInt_u   + e_u  *dt;
    eInt_psi = eInt_psi + e_psi*dt;
    eInt_phi = eInt_phi + e_phi*dt;
    eInt_u   = max(-intMax_u,   min(intMax_u,   eInt_u));
    eInt_psi = max(-intMax_psi, min(intMax_psi, eInt_psi));
    eInt_phi = max(-intMax_phi, min(intMax_phi, eInt_phi));

    TX = gains.Ku_p  *e_u   + gains.Ku_i  *eInt_u   - gains.Ku_d  *u;
    TN = gains.Kpsi_p*e_psi + gains.Kpsi_i*eInt_psi - gains.Kpsi_d*r;
    TK = gains.Kphi_p*e_phi + gains.Kphi_i*eInt_phi - gains.Kphi_d*p + TK_ff_bias;
    TY = 0;

    TX = max(-lims.TX, min(lims.TX, TX));
    TY = max(-lims.TY, min(lims.TY, TY));
    TN = max(-lims.TN, min(lims.TN, TN));
    TK = max(-lims.TK, min(lims.TK, TK));
    delta = 0.02*TN;
    thr   = TX;

    % ===== DISTURBANCE ANGIN (WIND) =====
    if wind.use
        [Xwind, Ywind, ~] = compute_wind_disturbance(u, v, psi, wind);
        TeU_wind = wind.kU * Xwind;
        TeV_wind = wind.kV * Ywind;
    else
        Xwind = 0; Ywind = 0;
        TeU_wind = 0; TeV_wind = 0;
    end
    Xwind_log = [Xwind_log; Xwind];
    Ywind_log = [Ywind_log; Ywind];

    Tcmd  = [TX; TY; delta; thr; TeU_wind; TeV_wind; TK; TN];

    % ===== INTEGRASI EULER =====
    [Vdot, eta_dot] = usv4dof(Vb, Tcmd, psi, phi, params);
    Vb  = Vb + Vdot*dt;
    x   = x   + eta_dot(1)*dt;
    y   = y   + eta_dot(2)*dt;
    psi = wrapToPi(psi + eta_dot(3)*dt);
    phi = wrapToPi(phi + eta_dot(4)*dt);

    traj    = [traj;   x y];
    x_log   = [x_log;  x];
    y_log   = [y_log;  y];
    psi_log = [psi_log; psi];
    phi_log = [phi_log; phi];

    [~, e_ct] = cte_by_projection(pFinal, [x y]);
    cte_log   = [cte_log; e_ct];

    dObs = inf;

for k = 1:size(obs_all,1)

    cx = obs_all(k,1);
    cy = obs_all(k,2);
    r0 = obs_all(k,3);

    dObs = min(dObs,...
    hypot(x-cx,y-cy) ...
    - r0 ...
    - avo.R_robot);

end
    dist_log    = [dist_log;    dObs];
    time_log    = [time_log;    t];
    t           = t + dt;
    x_des_log   = [x_des_log;   x_ref];
    y_des_log   = [y_des_log;   y_ref];
    psi_des_log = [psi_des_log; psi_des];
    phi_des_log = [phi_des_log; phi_des];
    beta_hat_log= [beta_hat_log; beta_hat];
    w_avo_log   = [w_avo_log;   w_avo];
    u_log       = [u_log;  Vb(1)];
    v_log       = [v_log;  Vb(2)];
    p_log       = [p_log;  Vb(3)];
    r_log       = [r_log;  Vb(4)];
    speed_log   = [speed_log; hypot(Vb(1),Vb(2))];

    set(hTraj,'XData',traj(:,1),'YData',traj(:,2));
    set(tKapal,'Matrix',makehgtform('translate',[x y 0],'zrotate',psi));
    if mod(round(t/dt),5)==0
    drawnow;
    pause(animPause);
end
end

dmin_traj_wp = min(hypot(traj(:,1)-waypoint(1), traj(:,2)-waypoint(2)));
fprintf('Trajektori: jarak minimum ke WP = %.3f m\n', dmin_traj_wp);

%% ===== Plot tambahan =====
figure(3); clf;

plot(time_log,dist_log,'b-','LineWidth',1.5);
grid on;

xlabel('Time (s)');
ylabel('Geometric Clearance (m)');
title('Minimum Geometric Clearance');

figure(4); clf;
set(gcf,'Name','Figure 4','NumberTitle','off','Color','w');
plot(time_log,abs(cte_log),'m-','LineWidth',1.5); grid on;
xlabel('Time (s)'); ylabel('|CTE| (m)');
title('Cross-Track Error vs Time');

%% ===== Figure 5 =====
fontAx=11; fontLab=12; fontTtl=14; lwA=1.8; lwD=1.4;
tA = time_log(:);
NA = min([numel(tA), numel(x_log), numel(y_log), numel(psi_log), numel(phi_log)]);
if NA > 0
    tA   = tA(1:NA);
    xA   = x_log(1:NA);
    yA   = y_log(1:NA);
    psiA = unwrap(psi_log(1:NA)) * 180/pi;
    phiA = unwrap(phi_log(1:NA)) * 180/pi;
    NDx  = min(NA, numel(x_des_log));
    NDy  = min(NA, numel(y_des_log));
    NDp  = min(NA, numel(psi_des_log));
    NDf  = min(NA, numel(phi_des_log));
    tDx  = tA(1:NDx);  xD   = x_des_log(1:NDx);
    tDy  = tA(1:NDy);  yD   = y_des_log(1:NDy);
    tDp  = tA(1:NDp);  psiD = unwrap(psi_des_log(1:NDp))*180/pi;
    tDf  = tA(1:NDf);  phiD = unwrap(phi_des_log(1:NDf))*180/pi;

    figStates = figure('Name','Figure 5','NumberTitle','off','Color','w');
    tl  = tiledlayout(figStates,4,1,'TileSpacing','compact','Padding','compact');
    axS = gobjects(4,1);
    labs = {'X [m]','Y [m]','\psi [deg]','\phi [deg]'};

    axS(1) = nexttile; hold(axS(1),'on'); box(axS(1),'on'); grid(axS(1),'on');
    set(axS(1),'FontSize',fontAx,'LineWidth',1);
    plot(tA,xA,'LineWidth',lwA,'DisplayName','Actual');
    if NDx>0, plot(tDx,xD,'--','LineWidth',lwD,'DisplayName','Desired'); end
    ylabel(labs{1},'FontSize',fontLab);

    axS(2) = nexttile; hold(axS(2),'on'); box(axS(2),'on'); grid(axS(2),'on');
    set(axS(2),'FontSize',fontAx,'LineWidth',1);
    plot(tA,yA,'LineWidth',lwA);
    if NDy>0, plot(tDy,yD,'--','LineWidth',lwD); end
    ylabel(labs{2},'FontSize',fontLab);

    axS(3) = nexttile; hold(axS(3),'on'); box(axS(3),'on'); grid(axS(3),'on');
    set(axS(3),'FontSize',fontAx,'LineWidth',1);
    plot(tA,psiA,'LineWidth',lwA);
    if NDp>0, plot(tDp,psiD,'--','LineWidth',lwD); end
    ylabel(labs{3},'FontSize',fontLab);

    axS(4) = nexttile; hold(axS(4),'on'); box(axS(4),'on'); grid(axS(4),'on');
    set(axS(4),'FontSize',fontAx,'LineWidth',1);
    plot(tA,phiA,'LineWidth',lwA);
    if NDf>0, plot(tDf,phiD,'--','LineWidth',lwD); end
    ylabel(labs{4},'FontSize',fontLab);
    xlabel('Time [s]','FontSize',fontLab);

    linkaxes(axS,'x');
    xlim(axS(1),[0 max(tA)]);
    lg = legend(axS(1),{'Actual','Desired'},'Orientation','horizontal','Location','southoutside');
    lg.Layout.Tile = 'south';
    title(tl,'States vs Time (Actual vs Desired)','FontSize',fontTtl,'FontWeight','bold');
end

if ~isempty(phi_log)
    fprintf('Maksimum |roll| = %.3f derajat\n', max(abs(rad2deg(phi_log))));
end

figure(6); clf;
set(gcf,'Name','Figure 6','NumberTitle','off','Color','w');
subplot(4,1,1); plot(time_log,u_log,'LineWidth',1.5);     grid on; ylabel('u (m/s)');
title('Body Velocities and Speed vs Time');
subplot(4,1,2); plot(time_log,v_log,'LineWidth',1.5);     grid on; ylabel('v (m/s)');
subplot(4,1,3); plot(time_log,speed_log,'LineWidth',1.5); grid on; ylabel('|v| (m/s)');
subplot(4,1,4); plot(time_log,r_log,'LineWidth',1.5);     grid on; ylabel('r (rad/s)');
xlabel('Time (s)');

figure(7); clf;
set(gcf,'Name','Figure 7','NumberTitle','off','Color','w');
plot(time_log,w_avo_log,'LineWidth',1.8); grid on;
ylim([-0.05 1.1]);
xlabel('Time (s)'); ylabel('w_{avo}');
title('Smooth ALOS <-> AVO blending weight (0=ALOS, 1=AVO)');

%% ==================== FUNGSI-FUNGSI ====================

function path = weightedAstar_grid_via(map, start_xy, wp_xy, goal_xy, epsilon, safeDist_m, cellSize)
maxR = size(map,1);  maxC = size(map,2);
sx = max(1,min(maxC, round(start_xy(1)/cellSize)));
sy = max(1,min(maxR, round(start_xy(2)/cellSize)));
wx = max(1,min(maxC, round(wp_xy(1)   /cellSize)));
wy = max(1,min(maxR, round(wp_xy(2)   /cellSize)));
gx = max(1,min(maxC, round(goal_xy(1) /cellSize)));
gy = max(1,min(maxR, round(goal_xy(2) /cellSize)));
distmap      = bwdist(map==1);
safeDistCell = max(1.0, safeDist_m/cellSize);
traversable  = distmap >= safeDistCell;
chk = @(x,y) ~(x<1||y<1||x>maxC||y>maxR) && traversable(y,x);
if ~chk(sx,sy), error('Start < safeDist.'); end
if ~chk(wx,wy), error('Waypoint < safeDist.'); end
if ~chk(gx,gy), error('Goal < safeDist.'); end
moves = [1 0;-1 0;0 1;0 -1;1 1;-1 -1;1 -1;-1 1];
costs = vecnorm(moves,2,2);
INF   = 1e9;
g = INF*ones(maxR,maxC,2);  f = g;
parentX  = zeros(maxR,maxC,2);
parentY  = parentX;
parentPh = parentX;
openMask = false(maxR,maxC,2);
visited  = false(maxR,maxC,2);
h_to_wp  = hypot((1:maxC)-wx,   (1:maxR)'-wy);
h_to_g   = hypot((1:maxC)-gx,   (1:maxR)'-gy);
g(sy,sx,1) = 0;
h0 = h_to_wp(sy,sx) + hypot(wx-gx,wy-gy);
f(sy,sx,1) = g(sy,sx,1) + epsilon*h0;
openMask(sy,sx,1) = true;
isWP = @(x,y)(x==wx && y==wy);
while any(openMask(:))
    fvals = f;  fvals(~openMask) = INF;
    [~,linIdx] = min(fvals(:));
    [cy,cx,cph] = ind2sub(size(fvals), linIdx);
    if (cx==gx && cy==gy && cph==2)
        path_cells = [cx cy];
        ph=cph; px=cx; py=cy;
        while ~(px==sx && py==sy && ph==1)
            nx = parentX(py,px,ph);  ny = parentY(py,px,ph);  nph = parentPh(py,px,ph);
            px=nx; py=ny; ph=nph;
            path_cells = [px py; path_cells];
        end
        path = [(path_cells(:,1)-0.5)*cellSize, (path_cells(:,2)-0.5)*cellSize];
        path(1,:)   = start_xy;
        path(end,:) = goal_xy;
        [~,idx_wp]  = min(vecnorm(path-wp_xy,2,2));
        path(idx_wp,:) = wp_xy;
        return;
    end
    openMask(cy,cx,cph) = false;
    visited(cy,cx,cph)  = true;
    for ii = 1:size(moves,1)
        nx=cx+moves(ii,1);  ny=cy+moves(ii,2);  nph=cph;
        if nx<1||ny<1||nx>maxC||ny>maxR, continue; end
        if ~traversable(ny,nx), continue; end
        if cph==1 && isWP(nx,ny), nph=2; end
        if visited(ny,nx,nph), continue; end
        tg = g(cy,cx,cph) + costs(ii);
        if tg < g(ny,nx,nph)
            g(ny,nx,nph) = tg;
            if nph==1
                h_lb = h_to_wp(ny,nx) + hypot(wx-gx,wy-gy);
            else
                h_lb = h_to_g(ny,nx);
            end
            f(ny,nx,nph)       = tg + epsilon*h_lb;
            parentX(ny,nx,nph) = cx;
            parentY(ny,nx,nph) = cy;
            parentPh(ny,nx,nph)= cph;
            openMask(ny,nx,nph)= true;
        end
    end
end
path = [];
end

function [Vdot, eta_dot] = usv4dof(V, T, psi, phi, P)
u=V(1); v=V(2); p=V(3); r=V(4);
Fx=T(1); Fy=T(2); delta=T(3); thr=T(4); %#ok<NASGU>
TeU=T(5); TeV=T(6); TePhi=T(7); TeR=T(8);
u_dot = P.A1*v*r + P.A2*u + P.A3*abs(u)*u + P.A4*(abs(u)^2)*u + P.A18*Fx + TeU;
v_dot = -(1/P.A1)*u*r + P.A5*v + P.A6*abs(v)*v + P.A7*(abs(v)^2)*v ...
        + P.A8*abs(r)*v + P.A9*abs(v)*r + TeV;
r_dot = -P.A10*v*u + P.A11*u*v + P.A12*r + P.A13*abs(r)*r + P.A14*(abs(r)^2)*r ...
        + P.A15*abs(r)*u + P.A16*abs(u)*r + P.A17*abs(u)*u ...
        + P.A20*abs(r)*u + P.A21*abs(u)*r + P.A22*abs(u)*u + P.A19*Fy + TeR;
p_dot = -P.KpLin*p - P.KpAbs*abs(p)*p - P.KpCub*(abs(p)^2)*p ...
        - P.Kphi*sin(phi) + P.Kfy*Fy + P.Kv*v + P.Kr*r ...
        + P.Kdelta*delta + P.Kbias + TePhi;
Vdot    = [u_dot; v_dot; p_dot; r_dot];
R       = [cos(psi), -sin(psi)*cos(phi), 0, 0;
           sin(psi),  cos(psi)*cos(phi), 0, 0;
           0,          0,                0, cos(phi);
           0,          0,                1, 0];
eta_dot = R * V;
end

function [Xwind, Ywind, Nwind] = compute_wind_disturbance(u, v, psi, wind)
    u_w = -wind.V_tw * cos(wind.beta_tw - psi);
    v_w = -wind.V_tw * sin(wind.beta_tw - psi);

    u_rw = u - u_w;
    v_rw = v - v_w;
    V_rw = hypot(u_rw, v_rw);

    Xwind = -0.5 * wind.rho_a * wind.Cdx * wind.A_Fw * V_rw * u_rw;
    Ywind = -0.5 * wind.rho_a * wind.Cdy * wind.A_Lw * V_rw * v_rw;
    Nwind = Ywind * (wind.Loa/4);   
end

function draw_wind_indicator(beta_tw, V_tw, mapW, mapH)
    cx = 0.08*mapW;
    cy = 0.90*mapH;
    L  = 0.07*mapW;
    ang_to = beta_tw + pi;
    dx = L*cos(ang_to);
    dy = L*sin(ang_to);
    quiver(cx, cy, dx, dy, 0, ...
        'Color',[0 0.45 0.85], 'LineWidth',2.2, ...
        'MaxHeadSize',2.0, 'HandleVisibility','off');
    text(cx - 0.05*mapW, cy + 0.06*mapH, ...
        sprintf('Angin %.1f m/s dari %.0f^{o}', V_tw, rad2deg(beta_tw)), ...
        'Color',[0 0.45 0.85], 'FontWeight','bold', 'FontSize',8, ...
        'HandleVisibility','off');
end

function cum = cumulativeArc(P)
if size(P,1)<2, cum=0;
else, cum = [0; cumsum(sqrt(sum(diff(P).^2,2)))]; end
end

function [s_on, remaining, projPoint, psi_id, y_e, last_idx] = ...
    projectArc_alos(P, cum, p, last_idx)
N = size(P,1);  best=inf;  s_on=0;  best_i=1;  best_proj=P(1,:);
i_start = max(1,   last_idx-3);
i_end   = min(N-1, last_idx+8);
for i = i_start:i_end
    A=P(i,:);  B=P(i+1,:);  AB=B-A;
    L2  = max(1e-12, dot(AB,AB));
    tt  = max(0, min(1, dot(p-A,AB)/L2));
    proj = A + tt*AB;
    d   = hypot(p(1)-proj(1), p(2)-proj(2));
    if d < best
        best=d;  best_i=i;  best_proj=proj;
        s_on = cum(i) + tt*sqrt(L2);
    end
end
last_idx  = best_i;
projPoint = best_proj;
remaining = cum(end) - s_on;
AB = P(best_i+1,:) - P(best_i,:);
if norm(AB)<1e-9, AB=[1 0]; end
psi_id = atan2(AB(2), AB(1));
Rot    = [cos(psi_id) sin(psi_id); -sin(psi_id) cos(psi_id)];
rel    = (p - projPoint).';
rel_s  = Rot * rel;
y_e    = rel_s(2);
end

function [segIdx, e_ct] = cte_by_projection(P, p)
n=size(P,1);  best=inf;  segIdx=2;
for i = 2:n
    p1=P(i-1,:);  p2=P(i,:);
    e = signedDistPointToSeg(p, p1, p2);
    if abs(e) < best
        best=abs(e);  segIdx=i;
    end
end
p1  = P(segIdx-1,:);
p2  = P(segIdx,:);
e_ct = signedDistPointToSeg(p, p1, p2);
end

function e = signedDistPointToSeg(p, a, b)
AB = b - a;
L2 = max(1e-12, dot(AB,AB));
e  = (AB(1)*(p(2)-a(2)) - AB(2)*(p(1)-a(1))) / max(1e-6, sqrt(L2));
end

function ang = wrapToPi(ang)
ang = atan2(sin(ang), cos(ang));
end

function Ps = smooth_path_g2cbs_c2(P, nPerSeg, epsRDP, maxCurv)
if nargin<2 || isempty(nPerSeg), nPerSeg = 90;  end
if nargin<3 || isempty(epsRDP),  epsRDP  = 0;   end
if nargin<4 || isempty(maxCurv), maxCurv = 0.6; end
P = remove_dups(P);
if size(P,1) <= 2, Ps = P; return; end

if epsRDP > 0 && size(P,1) > 3
    P = rdp(P, epsRDP); P = remove_dups(P);
    if size(P,1) <= 2, Ps = P; return; end
end
dt = 1 / max(2, nPerSeg);
Ps = g2cbs_planar(P, dt, maxCurv);
Ps = remove_dups(Ps);
end

function Ps = g2cbs_planar(P, dt, kappaMax)
clampFrac = 0.45;   
N = size(P,1);
if N < 3, Ps = P; return; end
c1 = 7.2364;
c2 = (2*(sqrt(6)-1))/5;
c3 = (c2+4)/(c1+6);
tBase = 0:dt:1;
tParm = [ (1-tBase).^3;          3*tBase.*(1-tBase).^2; ...
          3*tBase.^2.*(1-tBase); tBase.^3 ];     
tParmRev = flipud(tParm);
tParmRev = tParmRev(:,2:end);                      
X = P(1,1); Y = P(1,2);
for i = 1:N-2
    w1 = [P(i,:)  , 0].';
    w2 = [P(i+1,:), 0].';
    w3 = [P(i+2,:), 0].';
    Lin  = norm(w2-w1);  Lout = norm(w3-w2);
    if Lin < 1e-9 || Lout < 1e-9
        X = [X, w2(1)]; Y = [Y, w2(2)]; continue;
    end
    m1m2 = (w2-w1)/Lin;  m2m3 = (w3-w2)/Lout;
    gamma = acos( max(-1, min(1, m1m2.'*m2m3)) );
    % Frame lokal (ut menuju w2, normal ke arah w3) — sama dengan TA referensi
    ut = (w2-w1)/Lin;
    up = (w2-w3)/norm(w2-w3);
    ub = cross(up, ut); nb = norm(ub);
    % Guard kolinear: bila hampir lurus, lewati pembulatan (cegah pembagian nol/NaN)
    if nb < 1e-9 || gamma < 1e-4
        X = [X, w2(1)]; Y = [Y, w2(2)]; continue; %#ok<AGROW>
    end
    ub = ub/nb;  un = cross(ub, ut);
    R  = [ut, un, ub];                  % global <- lokal (R ortonormal)
    toL = @(w) R.'*(w - w1);            % global -> lokal
    m1 = toL(w1); m2 = toL(w2); m3 = toL(w3);
    u1 = -(m2-m1)/norm(m2-m1);
    u2 =  (m3-m2)/norm(m3-m2);
    beta = gamma/2; if beta == 0, beta = 1e-5; end
    
    d  = ((c2+4)^2/(54*c3)) * (sin(beta)/(kappaMax*cos(beta)^2));
    d  = min(d, clampFrac*min(Lin, Lout));   
    hb = c3*d;  gb = c2*c3*d;
    kb = ((6*c3*cos(beta))/(c2+4))*d;
    
    B0 = m2 + d*u1;  B1 = B0 - gb*u1;  B2 = B1 - hb*u1;
    E0 = m2 + d*u2;  E1 = E0 - gb*u2;  E2 = E1 - hb*u2;
    ud = (E2-B2)/norm(E2-B2);
    B3 = B2 + kb*ud;  E3 = E2 - kb*ud;
    
    Bm = [B0, B1, B2, B3];  Em = [E0, E1, E2, E3];
    toG = @(m) R*m + w1;                
    for j = 1:4, Bm(:,j) = toG(Bm(:,j)); Em(:,j) = toG(Em(:,j)); end
    curveB = Bm * tParm;                
    curveE = Em * tParmRev;
    X = [X, curveB(1,:), curveE(1,:)];  
    Y = [Y, curveB(2,:), curveE(2,:)];  
end
X = [X, P(end,1)];  Y = [Y, P(end,2)];
Ps = [X.', Y.'];
end

function [Pout, info] = enforce_safe_clearance(Pin, obs, safeDist, varargin)
ip = inputParser;
ip.addParameter('maxIter',80); ip.addParameter('gain',0.6);
ip.addParameter('maxStep',0.8); ip.addParameter('lambda',0.15);
ip.addParameter('ds',1.0); ip.addParameter('inflRad',2.0);
ip.addParameter('lambdaFar',0.0);
ip.parse(varargin{:});
prm = ip.Results;
radInfluence = prm.inflRad * safeDist;
P = resample_by_arclength(Pin, prm.ds);
N = size(P,1);
if N<=2, Pout=P; info.min_clearance=Inf; info.iterations=0; return; end
for it=1:prm.maxIter
    dP=zeros(N,2); vio=false(N,1);
    for i=2:N-1
        pi=P(i,:); push=[0 0]; clrMin = inf;
        for k=1:size(obs,1)
            c=obs(k,1:2); R=obs(k,3)+safeDist;
            vv=pi-c; d=norm(vv)+1e-9; clr=d-R;
            clrMin = min(clrMin, clr);
            buffer = 0.05;   
vv=pi-c; d=norm(vv)+1e-9; clr=d-R;
clrMin = min(clrMin, clr);
if clr < buffer                                  
    dir=vv/d; push=push+prm.gain*(buffer-clr)*dir; 
    if clr < 0, vio(i)=true; end
end
        end
        if clrMin <= 0, lambda_i = prm.lambda;
        elseif clrMin >= radInfluence, lambda_i = prm.lambdaFar;
        else
            w = (radInfluence - clrMin) / radInfluence;
            lambda_i = prm.lambdaFar + w*(prm.lambda - prm.lambdaFar);
        end
        smooth = lambda_i*((P(i-1,:)+P(i+1,:))/2 - P(i,:));
        dP(i,:) = push + smooth;
    end
    for i=2:N-1
        nrm=norm(dP(i,:));
        if nrm>prm.maxStep, dP(i,:)=dP(i,:)/nrm*prm.maxStep; end
    end
    P(2:N-1,:) = P(2:N-1,:) + dP(2:N-1,:);
    if ~any(vio), break; end
end
Pout=remove_dups(P);
info.iterations=it; info.min_clearance=min_clearance_poly(Pout, obs, safeDist);
end

function m = min_clearance_poly(P, obs, safeDist)
m=inf;
for k=1:size(obs,1)
    c=obs(k,1:2); R=obs(k,3)+safeDist;
    d = sqrt((P(:,1)-c(1)).^2 + (P(:,2)-c(2)).^2) - R;
    m = min(m, min(d));
end
end

function Q = resample_by_arclength(P, ds)
if size(P,1)<=2 || ds<=0, Q=P; return; end
s=cumulativeArc(P); ss=0:ds:s(end);
if ss(end)<s(end), ss=[ss s(end)]; end
Q=interp1(s,P,ss,'linear');
end

function Q = rdp(P, eps)
if size(P,1)<=2, Q=P; return; end
[d, idx] = maxPointDist(P);
if d > eps
    Q1=rdp(P(1:idx,:),eps); Q2=rdp(P(idx:end,:),eps);
    Q=[Q1(1:end-1,:); Q2];
else, Q=[P(1,:); P(end,:)]; end
end

function [dmax, idx] = maxPointDist(P)
A=P(1,:); B=P(end,:); AB=B-A;
L2=max(1e-12,sum(AB.^2)); dmax=-1; idx=1;
for i=2:size(P,1)-1
    AP=P(i,:)-A;
    tt=max(0,min(1,(AP*AB')/L2));
    proj=A+tt*AB; d=norm(P(i,:)-proj);
    if d>dmax, dmax=d; idx=i; end
end
if dmax<0, dmax=0; idx=1; end
end

function Q = remove_dups(Q)
if isempty(Q), return; end
keep=[true; vecnorm(diff(Q,1,1),2,2) > 1e-8];
Q=Q(keep,:);
end

function Pout = resmooth_keep_clearance(Pin, obs, safeDist, nPerSeg, alpha, iter, pinPts, maxCurv)
if nargin<5, alpha=0.5; end
if nargin<6, iter=8;   end
if nargin<7, pinPts=[]; end
if nargin<8 || isempty(maxCurv), maxCurv=0.6; end
Pcur = Pin; best = Pin;
bestClr = min_clearance_poly(Pin, obs, safeDist);
pinIdx = [];
if ~isempty(pinPts)
    for j=1:size(pinPts,1)
        [~, idx] = min(vecnorm(Pcur - pinPts(j,:), 2, 2));
        pinIdx = [pinIdx; idx]; 
        Pcur(idx,:) = pinPts(j,:);
    end
    pinIdx = unique(pinIdx(:));
end
for k=1:iter
    Sraw = smooth_path_g2cbs_c2(Pcur, nPerSeg, 0.5, maxCurv);   
    s_ref = cumulativeArc(Pcur);
    s_S   = cumulativeArc(Sraw);
    s_ref = min(max(s_ref, s_S(1)), s_S(end));
    S     = interp1(s_S, Sraw, s_ref, 'linear');
    if ~isempty(pinIdx)
        for j=1:numel(pinIdx)
            jp = min(j, size(pinPts,1));
            S(pinIdx(j),:)    = pinPts(jp,:);
            Pcur(pinIdx(j),:) = pinPts(jp,:);
        end
    end
    Ptry = (1-alpha)*S + alpha*Pcur;
    if ~isempty(pinIdx)
        for j=1:numel(pinIdx)
            jp = min(j, size(pinPts,1));
            Ptry(pinIdx(j),:) = pinPts(jp,:);
        end
    end
    clr = min_clearance_poly(Ptry, obs, safeDist);
    if clr >= 0
        Pcur = Ptry; best = Pcur; bestClr = clr;
        alpha = max(0.3, alpha*0.85);
    else
        alpha = alpha*0.5;
    end
    if alpha < 1e-3, break; end
end
Pout = best;
fprintf('Final re-smooth clearance = %.3f\n', bestClr);
end

function Pout = insert_support_points_around_wp(Pin, waypoint, beta)
if nargin<3, beta = 0.30; end
if isempty(Pin), Pout = Pin; return; end

max_r = max(abs(r_log));
max_p = max(abs(p_log));
max_v = max(abs(v_log));

fprintf('max r = %.3f rad/s\n', max_r);
fprintf('max p = %.3f rad/s\n', max_p);
fprintf('max v = %.3f m/s\n', max_v);

[~,j] = min(vecnorm(Pin - waypoint,2,2));
Pout = Pin;
if j>1 && j<size(Pin,1)
    prev = Pin(j-1,:); next = Pin(j+1,:);
    p_in  = (1-beta)*prev + beta*waypoint;
    p_out = (1-beta)*next + beta*waypoint;
    if norm(p_in - prev) > 1e-9 && norm(p_out - next) > 1e-9
        Pout = [Pin(1:j-1,:); p_in; waypoint; p_out; Pin(j+1:end,:)];
    else
        Pout = [Pin(1:j-1,:); waypoint; Pin(j+1:end,:)];
    end
else
    Pout = Pin;
end
Pout = remove_dups(Pout);
end

function meanKappa = computeMeanCurvature(path)

if size(path,1) < 3
    meanKappa = 0;
    return;
end

kappa = [];

for i = 2:size(path,1)-1

    p1 = path(i-1,:);
    p2 = path(i,:);
    p3 = path(i+1,:);

    a = norm(p2-p1);
    b = norm(p3-p2);
    c = norm(p3-p1);

    if a<1e-6 || b<1e-6 || c<1e-6
        continue;
    end

    A = abs(det([p2-p1; p3-p1]))/2;

    k = 4*A/(a*b*c);

    kappa(end+1) = k;

end

meanKappa = mean(kappa);

end

function [v_xy, sgn] = avo_select_velocity_xy(pA, vA, v_pref, pB, vB, R_obs, R_bot, A, d_cc, sgn_locked)
r=pB-pA;  d=norm(r);
if d<1e-6, r=[1;0];  d=1e-6; end
r_hat      = r/d;
v_rel_pref = v_pref - vB;
vmag_pref  = norm(v_rel_pref);
if vmag_pref<1e-9, v_xy=v_pref; return; end
v_rel_now = vA - vB;
proj_rel  = max(0, dot(v_rel_now, r_hat));

R_clear = R_obs + R_bot + A.safe_dist;

R_exp_raw = R_clear + A.gamma*proj_rel / max(d^A.lambda,1e-6);

R_exp = min(R_exp_raw, A.max_expand_ratio * R_clear);

global Rexp_log Rclear_log
Rexp_log(end+1)   = R_exp;
Rclear_log(end+1) = R_clear;

ratio_raw = R_exp/d;                 
ratio     = clampv(ratio_raw, 0, 0.999);
alpha     = asin(ratio);

global ratio_log alpha_deg_log
ratio_log(end+1)     = ratio_raw;
alpha_deg_log(end+1) = rad2deg(alpha);

phi_r     = atan2(r_hat(2), r_hat(1));
phi_pref  = atan2(v_rel_pref(2), v_rel_pref(1));
dotp      = clampv(dot(v_rel_pref/vmag_pref, r_hat), -1, 1);
ang       = acos(dotp);
sgn_raw   = sign(r_hat(1)*v_rel_pref(2) - r_hat(2)*v_rel_pref(1));
if sgn_raw==0, sgn_raw=1; end
if sgn_locked == 0
    sgn = sgn_raw;      % awal encounter -> tentukan sisi hindar (kiri/kanan)
else
    sgn = sgn_locked;   % sudah komit -> pertahankan sisi ini sampai obstacle terlewati
end
margin = deg2rad(3) + deg2rad(5)*(R_exp-R_clear)/(R_exp+eps);
phi_bd = phi_r + sgn*(alpha + margin);
if (ang<=alpha) && (dot(v_rel_pref,r_hat)>0)
    depth   = clampv((alpha-ang)/max(alpha,1e-6), 0, 1);
    w       = depth^2;
    dphi    = wrapToPi(phi_bd-phi_pref) + sgn*A.bias_boundary;
    phi_cmd = phi_pref + w*dphi;
else
    slack = ang - alpha;
    if slack<A.soft_alpha && dot(v_rel_pref,r_hat)>0
        w       = smoothstep01(1-slack/A.soft_alpha);
        phi_cmd = phi_pref + sgn*(A.bias_boundary*w);
    else
        phi_cmd = phi_pref;
    end
end
vmag    = clampv(norm(v_pref), A.min_speed, A.max_speed);
R_clear = max(R_clear, 1e-6);

far     = max(A.activate_R-R_exp, 1e-6);
scale   = clampv(0.55+0.45*((d_cc-R_exp)/far), 0.55, 1.0);
global scale_log

scale_log(end+1) = scale;
vmag    = vmag*scale;
v_rel_new = vmag*[cos(phi_cmd); sin(phi_cmd)];
v_xy      = vB + v_rel_new;
v_xy      = real(v_xy);
if ~all(isfinite(v_xy)), v_xy=v_pref; end
end

function y = clampv(x, lo, hi)
y = min(max(x,lo), hi);
end

function s = smoothstep01(u)
u = clampv(u,0,1);
s = u*u*(3-2*u);
end

function ang = blend_angles(a, b, w)
d   = atan2(sin(b-a), cos(b-a));
ang = atan2(sin(a+w*d), cos(a+w*d));
end

clrSmooth = min_clearance_poly(pSmooth0, obstacles, safeDistPlan);
clrFinal  = min_clearance_poly(pFinal,   obstacles, safeDistPlan);

fprintf('pSmooth0 clearance = %.4f\n', clrSmooth);
fprintf('pFinal   clearance = %.4f\n', clrFinal);

fprintf('\n===== %s PERFORMANCE =====\n', algoLabel);
fprintf('Mean CTE = %.4f m\n', mean(abs(cte_log)));
fprintf('RMS  CTE = %.4f m\n', sqrt(mean(cte_log.^2)));
fprintf('Max  CTE = %.4f m\n', max(abs(cte_log)));
fprintf('Min Clearance = %.4f m\n', min(dist_log));
fprintf('Max Roll = %.4f deg\n', max(abs(rad2deg(phi_log))));
fprintf('==========================\n');

phi_err = rad2deg(phi_log - phi_des_log);

rmse_phi = sqrt(mean(phi_err.^2));
max_phi_err = max(abs(phi_err));

fprintf('RMSE Roll Error = %.3f deg\n', rmse_phi);
fprintf('Max Roll Error  = %.3f deg\n', max_phi_err);

figure;
plot(time_log, rad2deg(phi_des_log),'r--','LineWidth',2);
hold on;
plot(time_log, rad2deg(phi_log),'b','LineWidth',1.5);
grid on;
legend('Desired','Actual');
xlabel('Time [s]');
ylabel('Roll [deg]');

figure;
plot(time_log, rad2deg(r_log));
grid on;
ylabel('Yaw Rate [deg/s]');

figure;
plot(time_log, rad2deg(psi_log),'b');
hold on;
plot(time_log, rad2deg(psi_des_log),'r--');
grid on;
legend('Actual','Desired');
title('Heading Tracking');

%% ===== DISTURBANCE ANGIN (WIND) =====
figure;
plot(time_log, Xwind_log, 'b', 'LineWidth', 1.5); hold on;
plot(time_log, Ywind_log, 'm', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Gaya Angin [N]');
legend('X_{wind} (surge)','Y_{wind} (sway)');
title(sprintf('Disturbance Angin (V_{tw}=%.1f m/s, \\beta_{tw}=%.0f^{o})', ...
    wind.V_tw, rad2deg(wind.beta_tw)));

fprintf('\n===== DISTURBANCE ANGIN =====\n');
fprintf('wind.use      = %d\n', wind.use);
fprintf('V_tw          = %.2f m/s\n', wind.V_tw);
fprintf('beta_tw       = %.1f deg\n', rad2deg(wind.beta_tw));
fprintf('Mean |Xwind|  = %.4f N\n', mean(abs(Xwind_log)));
fprintf('Max  |Xwind|  = %.4f N\n', max(abs(Xwind_log)));
fprintf('Mean |Ywind|  = %.4f N\n', mean(abs(Ywind_log)));
fprintf('Max  |Ywind|  = %.4f N\n', max(abs(Ywind_log)));
fprintf('==============================\n');

%% ===== DISTANCE TO STATIC OBSTACLES =====
distStatic = inf(size(time_log));

for i = 1:length(time_log)

    x = x_log(i);
    y = y_log(i);

    dmin = inf;

    for k = 1:size(obstacles,1)

        cx = obstacles(k,1);
        cy = obstacles(k,2);
        r  = obstacles(k,3);

        d = hypot(x-cx,y-cy) - r;

        dmin = min(dmin,d);
    end

    distStatic(i) = dmin;
end

figure;
plot(time_log,distStatic,'b','LineWidth',2);
hold on;
yline(safeDist,'r--','Minimum Safe Distance');

grid on;
xlabel('Time [s]');
ylabel('Distance [m]');
title('USV Distance to Static Obstacles');
legend('Static Obstacles','Safe Distance');

%% ===== DISTANCE TO DYNAMIC OBSTACLES =====

distDyn1 = zeros(size(time_log));
distDyn2 = zeros(size(time_log));

for i = 1:length(time_log)

    x = x_log(i);
    y = y_log(i);

    % posisi obstacle dinamis saat itu
    y1 = dyn1.y0 + dyn1.v*time_log(i);
    y2 = dyn2.y0 - dyn2.v*time_log(i);

    distDyn1(i) = hypot(x-dyn1.x , y-y1) - dyn1.R;
    distDyn2(i) = hypot(x-dyn2.x , y-y2) - dyn2.R;
end

figure;
plot(time_log,distDyn1,'b','LineWidth',2);
hold on;
plot(time_log,distDyn2,'m','LineWidth',2);

yline(safeDist,'r--','Minimum Safe Distance');

grid on;
xlabel('Time [s]');
ylabel('Distance [m]');
title('USV Distance to Dynamic Obstacles');

legend('Dynamic Obs 1',...
       'Dynamic Obs 2',...
       'Safe Distance');

fprintf('safeDist = %.3f\n',safeDist);
fprintf('avo.safe_dist = %.3f\n',avo.safe_dist);
fprintf('R_robot = %.3f\n',avo.R_robot);
fprintf('dynR = %.3f\n',dynR);
fprintf('\n===== DISTANCE SUMMARY =====\n');

fprintf('Min Static Distance = %.3f m\n', ...
        min(distStatic));

fprintf('Min Dynamic Obs 1 Distance = %.3f m\n', ...
        min(distDyn1));

fprintf('Min Dynamic Obs 2 Distance = %.3f m\n', ...
        min(distDyn2));

%% ===== Distance to EACH Static Obstacle =====

Nobs = size(obstacles,1);

distStaticAll = zeros(length(time_log),Nobs);

for i = 1:length(time_log)

    x_usv = x_log(i);
    y_usv = y_log(i);

    for k = 1:Nobs

        cx = obstacles(k,1);
        cy = obstacles(k,2);
        r  = obstacles(k,3);

        distStaticAll(i,k) = ...
            hypot(x_usv-cx,y_usv-cy) - r;

    end
end

figure;
hold on;
grid on;

for k = 1:Nobs
    plot(time_log,...
         distStaticAll(:,k),...
         'LineWidth',1.5,...
         'DisplayName',sprintf('Obs %d',k));
end

yline(safeDist,'r--','Safe Distance');

xlabel('Time [s]');
ylabel('Distance [m]');
title('USV Distance to Each Static Obstacle');

legend('Location','eastoutside');

%% ===== SIMPAN HASIL & TABEL PERBANDINGAN VO vs AVO =====
resFile = fullfile(pwd,'hasil_perbandingan_VO_AVO.mat');

% ----- diagnostik saturasi ratio & clipping turn (baru) -----
if isempty(ratio_log)
    satFrac = NaN; meanRatio = NaN; maxRatio = NaN; meanAlphaDeg = NaN; maxAlphaDeg = NaN;
else
    satFrac      = mean(ratio_log >= 0.999) * 100;   % % langkah dgn R/d >= clamp
    meanRatio    = mean(ratio_log);
    maxRatio     = max(ratio_log);
    meanAlphaDeg = mean(alpha_deg_log);
    maxAlphaDeg  = max(alpha_deg_log);
end

if isempty(turn_clipped_log)
    clipFrac = NaN; meanTurnReqDeg = NaN; maxTurnReqDeg = NaN;
else
    clipFrac       = mean(turn_clipped_log) * 100;   % % langkah dgn |dpsi| > 50 deg
    meanTurnReqDeg = mean(abs(turn_req_deg_log));
    maxTurnReqDeg  = max(abs(turn_req_deg_log));
end

if isempty(scale_log)
    meanScale = NaN; minScale = NaN;
else
    meanScale = mean(scale_log);
    minScale  = min(scale_log);
end

hasilBaru = struct( ...
    'MeanCTE',        mean(abs(cte_log)), ...
    'RMSCTE',         sqrt(mean(cte_log.^2)), ...
    'MaxCTE',         max(abs(cte_log)), ...
    'MinDistObs',     min(dist_log), ...
    'MaxRoll',        max(abs(rad2deg(phi_log))), ...
    'SatFracPct',     satFrac, ...
    'MeanRatio',      meanRatio, ...
    'MaxRatio',       maxRatio, ...
    'MeanAlphaDeg',   meanAlphaDeg, ...
    'MaxAlphaDeg',    maxAlphaDeg, ...
    'ClipFracPct',    clipFrac, ...
    'MeanTurnReqDeg', meanTurnReqDeg, ...
    'MaxTurnReqDeg',  maxTurnReqDeg, ...
    'MeanScale',      meanScale, ...
    'MinScale',       minScale, ...
    'SwitchDuration', sum(w_avo_log > 0.05) * dt );

if isfile(resFile)
    S = load(resFile);
else
    S = struct();
end
S.(algoLabel) = hasilBaru;
save(resFile, '-struct', 'S');

fprintf('\nHasil mode %s tersimpan ke: %s\n', algoLabel, resFile);

if isfield(S,'VO') && isfield(S,'AVO')
    fprintf('\n================= TABEL PERBANDINGAN KINERJA VO vs AVO =================\n');
    fprintf('%-32s %12s %12s\n','Parameter','VO','AVO');
    fprintf('%-32s %12.4f %12.4f\n','Mean CTE (m)',                S.VO.MeanCTE,    S.AVO.MeanCTE);
    fprintf('%-32s %12.4f %12.4f\n','RMS CTE (m)',                 S.VO.RMSCTE,     S.AVO.RMSCTE);
    fprintf('%-32s %12.4f %12.4f\n','Max CTE (m)',                 S.VO.MaxCTE,     S.AVO.MaxCTE);
    fprintf('%-32s %12.4f %12.4f\n','Minimum Distance to Obstacle (m)', S.VO.MinDistObs, S.AVO.MinDistObs);
    fprintf('%-32s %12.2f %12.2f\n','Maximum Roll (deg)',          S.VO.MaxRoll,    S.AVO.MaxRoll);
    fprintf('==========================================================================\n');

    fprintf('\n========= DIAGNOSTIK SATURASI / CLIPPING (kenapa AVO bisa kalah) =========\n');
    fprintf('%-32s %12s %12s\n','Parameter','VO','AVO');
    fprintf('%-32s %12.2f %12.2f\n','%% langkah ratio jenuh (R/d>=0.999)', S.VO.SatFracPct,   S.AVO.SatFracPct);
    fprintf('%-32s %12.3f %12.3f\n','Mean R/d ratio',                     S.VO.MeanRatio,    S.AVO.MeanRatio);
    fprintf('%-32s %12.3f %12.3f\n','Max  R/d ratio',                     S.VO.MaxRatio,     S.AVO.MaxRatio);
    fprintf('%-32s %12.2f %12.2f\n','Mean alpha (deg)',                   S.VO.MeanAlphaDeg, S.AVO.MeanAlphaDeg);
    fprintf('%-32s %12.2f %12.2f\n','Max  alpha (deg)',                   S.VO.MaxAlphaDeg,  S.AVO.MaxAlphaDeg);
    fprintf('%-32s %12.2f %12.2f\n','%% langkah turn ke-clip (>50deg)',   S.VO.ClipFracPct,  S.AVO.ClipFracPct);
    fprintf('%-32s %12.2f %12.2f\n','Mean |turn diminta| (deg)',          S.VO.MeanTurnReqDeg, S.AVO.MeanTurnReqDeg);
    fprintf('%-32s %12.2f %12.2f\n','Max  |turn diminta| (deg)',          S.VO.MaxTurnReqDeg,  S.AVO.MaxTurnReqDeg);
    fprintf('%-32s %12.3f %12.3f\n','Mean speed scale',                   S.VO.MeanScale,    S.AVO.MeanScale);
    fprintf('%-32s %12.3f %12.3f\n','Min  speed scale',                   S.VO.MinScale,     S.AVO.MinScale);
    fprintf('%-32s %12.2f %12.2f\n','Switch/avoidance duration (s)',      S.VO.SwitchDuration, S.AVO.SwitchDuration);
    fprintf('===========================================================================\n');
    fprintf(['Cara baca: kalau kolom AVO jauh lebih besar di "%% ratio jenuh" dan\n' ...
             '"%% turn ke-clip" dibanding VO, itu konfirmasi bahwa R_exp (AVO) sering\n' ...
             'meledak jauh melebihi jarak sebenarnya -> alpha mentok ~87 derajat ->\n' ...
             'sudut belok yang diminta ke-clip di 50 derajat -> AVO jadi "selalu belok\n' ...
             'maksimum" alih-alih modulasi halus, sambil scale kecepatan ikut turun ke\n' ...
             'titik minimum (0.55) di saat yang sama -> CTE & roll naik TAPI clearance\n' ...
             'malah tidak lebih baik dari VO karena eksekusinya kepotong dan lebih lambat.\n']);
else
    fprintf('Jalankan sekali lagi dengan useAdaptiveVO = %s untuk melengkapi tabel perbandingan.\n', ...
        mat2str(~useAdaptiveVO));
end

%% ===== SIMPAN HASIL & TABEL PERBANDINGAN NO-WIND vs WIND =====
if wind.use
    windLabel = 'Wind';
else
    windLabel = 'NoWind';
end

resFileWind = fullfile(pwd,'hasil_perbandingan_Wind.mat');

hasilWindBaru = struct( ...
    'MeanCTE',    mean(abs(cte_log)), ...
    'RMSCTE',     sqrt(mean(cte_log.^2)), ...
    'MaxCTE',     max(abs(cte_log)), ...
    'MinDistObs', min(dist_log), ...
    'MaxRoll',    max(abs(rad2deg(phi_log))), ...
    'MeanXwind',  mean(abs(Xwind_log)), ...
    'MeanYwind',  mean(abs(Ywind_log)) );

if isfile(resFileWind)
    Sw = load(resFileWind);
else
    Sw = struct();
end
Sw.(windLabel) = hasilWindBaru;
save(resFileWind, '-struct', 'Sw');

fprintf('\nHasil mode %s tersimpan ke: %s\n', windLabel, resFileWind);

if isfield(Sw,'NoWind') && isfield(Sw,'Wind')
    fprintf('\n================= TABEL PERBANDINGAN KINERJA NO-WIND vs WIND =================\n');
    fprintf('%-32s %12s %12s\n','Parameter','No-Wind','Wind');
    fprintf('%-32s %12.4f %12.4f\n','Mean CTE (m)',                Sw.NoWind.MeanCTE,    Sw.Wind.MeanCTE);
    fprintf('%-32s %12.4f %12.4f\n','RMS CTE (m)',                 Sw.NoWind.RMSCTE,     Sw.Wind.RMSCTE);
    fprintf('%-32s %12.4f %12.4f\n','Max CTE (m)',                 Sw.NoWind.MaxCTE,     Sw.Wind.MaxCTE);
    fprintf('%-32s %12.4f %12.4f\n','Minimum Distance to Obstacle (m)', Sw.NoWind.MinDistObs, Sw.Wind.MinDistObs);
    fprintf('%-32s %12.2f %12.2f\n','Maximum Roll (deg)',          Sw.NoWind.MaxRoll,    Sw.Wind.MaxRoll);
    fprintf('%-32s %12.4f %12.4f\n','Mean |Xwind| (N)',            Sw.NoWind.MeanXwind,  Sw.Wind.MeanXwind);
    fprintf('%-32s %12.4f %12.4f\n','Mean |Ywind| (N)',            Sw.NoWind.MeanYwind,  Sw.Wind.MeanYwind);
    fprintf('================================================================================\n');
else
    fprintf('Jalankan sekali lagi dengan wind.use = %s untuk melengkapi tabel perbandingan No-Wind vs Wind.\n', ...
        mat2str(~wind.use));
end
trajLength = sum(vecnorm(diff(traj),2,2));

fprintf('\n===== TRAJECTORY =====\n');
fprintf('Trajectory Length = %.3f m\n', trajLength);

travelTime = t;

fprintf('Travel Time = %.2f s\n', travelTime);

switchDuration = sum(w_avo_log > 0.05) * dt;

fprintf('Switching Duration = %.2f s\n', switchDuration);

fprintf('\n===== AVO RADIUS =====\n');

fprintf('Mean R_clear = %.3f\n',mean(Rclear_log));
fprintf('Mean R_exp   = %.3f\n',mean(Rexp_log));

fprintf('Max  R_exp   = %.3f\n',max(Rexp_log));
fprintf('Min  R_exp   = %.3f\n',min(Rexp_log));

figure;

plot(Rclear_log,'b','LineWidth',1.5);
hold on;
plot(Rexp_log,'r','LineWidth',1.5);

grid on;

legend('R_{clear}','R_{exp}');
title('Adaptive Radius');
xlabel('Iteration');
ylabel('Radius (m)');

fprintf('Mean scale = %.3f\n',mean(scale_log));
fprintf('Min  scale = %.3f\n',min(scale_log));
fprintf('Max  scale = %.3f\n',max(scale_log));

figure;
plot(scale_log,'LineWidth',1.5);
grid on;
xlabel('Iteration');
ylabel('Scale');
title('Adaptive Speed Scaling');