%% SEDS_cascade
% Simulation interactive en temps reel du quadricoptere ESP-Drone.

% Fermez une des 3 fenetres pour arreter la simulation.



clc;
clear;
close all;

rng(1,'twister');   % Initialise le gÃ©nÃ©rateur alÃ©atoire

%% Parametres physiques ESP-Drone 
m = 0.053;        % kg
G = 9.81;         % m/s^2
L_emp = 0.10;     % empattement moteur-moteur [m]
l = L_emp/(2*sqrt(2));
Ix = 0.5*m*l^2;
Iy = Ix;
Iz = Ix + Iy;

fprintf('Parametres utilises : m=%.4f kg, l=%.4f m, Ix=%.3e, Iy=%.3e, Iz=%.3e\n', m, l, Ix, Iy, Iz);

%% Temps de simulation
dt = 0.02;     % pas de temps [s] (50 Hz)
t  = 0;

%% Consignes de vol stationnaire
x_ref = 0; y_ref = 0; z_ref = 1; psi_ref = 0;

%% Gains PID initiaux, regroupes par boucle de contrôle
% (modifiables en direct via les sliders de la figure principale)
sim_data = struct();
Kpz = 3.0;Kiz = 0.02;Kdz = 3.8;
%sim_data.Kpz    = 2.8;   sim_data.Kiz    = 0.05;    sim_data.Kdz    = 3.4;    % Altitude (z) - reglage doux
sim_data.Kpz    = 9.5;   sim_data.Kiz    = 0.003;    sim_data.Kdz    = 7.0;    % Altitude (z) - reglage doux
%sim_data.Kpz    =  5.5;   sim_data.Kiz    = 0.01;    sim_data.Kdz    =5.2;;    % Altitude (z) - reglage doux
sim_data.Kp_pos = 0.1;  sim_data.Ki_pos = 5e-4;   sim_data.Kd_pos = 0.14;   % Position (x,y)
%sim_data.Kp_att = 2e-4;  sim_data.Ki_att = 0;      sim_data.Kd_att = 1e-4;  % Attitude (phi,theta,psi)
% Réglage rigoureux attitude : Tr = 0.5 s, zeta = 0.8
zeta_att = 0.8;
Tr_att = 0.5;
wn_att = 4/(zeta_att*Tr_att);

% Roll
sim_data.Kp_phi = Ix*wn_att^2;
sim_data.Ki_phi = 0;
sim_data.Kd_phi = 2*zeta_att*Ix*wn_att;

% Pitch
sim_data.Kp_theta = Iy*wn_att^2;
sim_data.Ki_theta = 0;
sim_data.Kd_theta = 2*zeta_att*Iy*wn_att;

% Yaw
sim_data.Kp_psi = Iz*wn_att^2;
sim_data.Ki_psi = 0;
sim_data.Kd_psi = 2*zeta_att*Iz*wn_att;

%% Limites / anti-windup 
angle_sat   = 0.25;       % saturation des references phi_ref/theta_ref [rad]
angle_safe  = deg2rad(60);% sécurite supplémentaire sur les angles réels (régulation interactive)
U1_max = 3*m*G;  U1_min = 0;
int_lim_pos = 2;  int_lim_z = 2;  int_lim_ang = 0.5;

%% Etats initiaux
x=0; y=0; z=0; vx=0; vy=0; vz=0;
phi=0; theta=0; psi=0; p=0; q=0; r=0;

%% Intégrales des erreurs
int_z=0; int_x=0; int_y=0; int_phi=0; int_theta=0; int_psi=0;
last_reset = 0; reset_interval = 6;   % reset periodique anti-windup

%% Vent : rafales aléatoires générées en continu
wind_active = false; wind_start = 0; wind_duration = 0;
wind_speed_kmh = 0; wind_dir = 0;
wind_x = 0; wind_y = 0;
next_wind_time = rand*2 + 2;          % première rafale entre 2 et 4 s

%% Indicateurs de robustesse
nb_rafales = 0;
max_abs_ex = 0;
max_abs_ey = 0;
max_abs_ez = 0;
z_min = z_ref;
z_max = z_ref;
instabilite = false;
t_eval_start = 5;   % début de l'évaluation de robustesse après le décollage

% Seuils utilisés pour qualifier la robustesse
seuil_xy = 0.15;     % erreur horizontale maximale acceptable [m]
seuil_z  = 0.05;     % erreur verticale maximale acceptable [m]
z_lim_min = 0.50;    % limite basse de sécurité [m]
z_lim_max = 1.50;    % limite haute de sécurité [m]


%% Buffers pour les graphiques (fenêtre temporelle glissante)
T_window = 10;
t_vec = [];
Pz_vec=[]; Iz_vec=[]; Dz_vec=[];
Px_vec=[]; Iy_vec_g=[]; Dx_vec=[];
Pphi_vec=[]; Iphi_vec=[]; Dphi_vec=[];
ex_vec=[]; ey_vec=[]; ez_vec=[];
ephi_vec=[]; etheta_vec=[]; epsi_vec=[];
xtraj=[]; ytraj=[]; ztraj=[];   % tracé de la trajectoire (fenêtre glissante)

%% ================= FIGURE 1 : Animation 3D + sliders =================
fig_drone = figure('Position',[50 40 1050 720],'Color','w', ...
    'Name','Simulation Quadricoptere ESP-Drone (interactif)');
ax = axes('Parent',fig_drone,'Units','pixels','Position',[40 95 620 580]);
hold(ax,'on'); grid(ax,'on'); view(ax,45,28);
xlabel(ax,'x [m]'); ylabel(ax,'y [m]'); zlabel(ax,'z [m]');
axis(ax,[-0.7 0.7 -0.7 0.7 0 1.6]);
axis(ax,'manual');
axis(ax,'vis3d');
pbaspect(ax,[1 1 0.9]);
view(ax,45,28);
camproj(ax,'perspective');
title(ax,'Simulation Quadricoptere');

h_body = plot3(ax,0,0,0,'ko','MarkerFaceColor','k','MarkerSize',7);
h_armX = plot3(ax,[0 0],[0 0],[0 0],'r-','LineWidth',3);
h_armY = plot3(ax,[0 0],[0 0],[0 0],'b-','LineWidth',3);
h_mot  = plot3(ax,zeros(1,4),zeros(1,4),zeros(1,4),'ko','MarkerFaceColor',[0.8 0.8 0.8],'MarkerSize',6);
h_traj = plot3(ax,NaN,NaN,NaN,'k--','LineWidth',1.1);
h_ref  = plot3(ax,x_ref,y_ref,z_ref,'g*','MarkerSize',12,'LineWidth',2);
h_wind = quiver3(ax,0,0,1,0,0,0,'m','LineWidth',2,'MaxHeadSize',0.7);
h_wind_label = text(ax,-0.65,-0.65,0,'Vent : 0.0 km/h, Direction : 0 deg','FontSize',9);

% Affichage numérique des performances en altitude
h_metrics = uicontrol('Parent',fig_drone,'Style','text','Position',[40 60 620 28], ...
    'String','z=0.000 m | e_z=1.000 m | Depassement=0.00 % | Tr95=-- s', ...
    'FontWeight','bold','HorizontalAlignment','left','BackgroundColor','w');

Lvis = 0.16;   % longueur visuelle des bras

% ---------- Sliders compacts a droite : 5 groupes (Altitude / Position / Roll / Pitch / Yaw) ----------
rows = {
    'Altitude (z)',        'Kpz',      'Kiz',      'Kdz',      0,   10,    0, 2,    0,    5
    'Position (x,y)',      'Kp_pos',   'Ki_pos',   'Kd_pos',   0,   0.5,   0, 0.01, 0,    0.5
    'Roll (phi)',          'Kp_phi',   'Ki_phi',   'Kd_phi',   0,   1e-2,  0, 5e-4, 0,    2e-3
    'Pitch (theta)',       'Kp_theta', 'Ki_theta', 'Kd_theta', 0,   1e-2,  0, 5e-4, 0,    2e-3
    'Yaw (psi)',           'Kp_psi',   'Ki_psi',   'Kd_psi',   0,   1e-2,  0, 5e-4, 0,    2e-3
};

xPanel = 690;
yBase  = [610 500 390 280 170];
for rIdx = 1:5
    rowLabel = rows{rIdx,1};
    fields   = rows(rIdx,2:4);
    minKp = rows{rIdx,5};  maxKp = rows{rIdx,6};
    minKi = rows{rIdx,7};  maxKi = rows{rIdx,8};
    minKd = rows{rIdx,9};  maxKd = rows{rIdx,10};
    minv  = [minKp minKi minKd];
    maxv  = [maxKp maxKi maxKd];
    val0  = [sim_data.(fields{1}), sim_data.(fields{2}), sim_data.(fields{3})];
    val0  = max(min(val0,maxv),minv);

    uicontrol('Parent',fig_drone,'Style','text','Position',[xPanel yBase(rIdx)+65 330 18], ...
        'String',rowLabel,'FontWeight','bold','HorizontalAlignment','left','BackgroundColor','w');

    shortNames = {'Kp','Ki','Kd'};
    for c = 1:3
        yLine = yBase(rIdx) + 65 - 24*c;
        uicontrol('Parent',fig_drone,'Style','text','Position',[xPanel yLine+2 30 18], ...
            'String',shortNames{c},'HorizontalAlignment','left','BackgroundColor','w');
        txt = uicontrol('Parent',fig_drone,'Style','text','Position',[xPanel+300 yLine+2 85 18], ...
            'String',sprintf('%.6g',val0(c)),'HorizontalAlignment','left','BackgroundColor','w');
        uicontrol('Parent',fig_drone,'Style','slider','Position',[xPanel+35 yLine 255 20], ...
            'Min',minv(c),'Max',maxv(c),'Value',val0(c), ...
            'Callback',@(src,~) localSliderCallback(fig_drone, fields{c}, txt, src));
    end
end
set(fig_drone,'UserData',sim_data);

%% ================= FIGURE 2 : Graphiques des termes PID =================
fig_pid = figure('Position',[660 280 600 480],'Color','w','Name','Termes PID (temps reel)');
subplot(3,3,1); h_Pz   = plot(NaN,NaN,'r-','LineWidth',1.6); title('P altitude'); ylabel('P_z'); grid on;
subplot(3,3,4); h_Iz   = plot(NaN,NaN,'g-','LineWidth',1.6); title('I altitude'); ylabel('I_z'); grid on;
subplot(3,3,7); h_Dz   = plot(NaN,NaN,'b-','LineWidth',1.6); title('D altitude'); ylabel('D_z'); xlabel('Temps (s)'); grid on;
subplot(3,3,2); h_Px   = plot(NaN,NaN,'r-','LineWidth',1.6); title('P position (x)'); ylabel('P_x'); grid on;
subplot(3,3,5); h_Ix   = plot(NaN,NaN,'g-','LineWidth',1.6); title('I position (x)'); ylabel('I_x'); grid on;
subplot(3,3,8); h_Dx   = plot(NaN,NaN,'b-','LineWidth',1.6); title('D position (x)'); ylabel('D_x'); xlabel('Temps (s)'); grid on;
subplot(3,3,3); h_Pphi = plot(NaN,NaN,'r-','LineWidth',1.6); title('P attitude (\phi)'); ylabel('P_\phi'); grid on;
subplot(3,3,6); h_Iphi = plot(NaN,NaN,'g-','LineWidth',1.6); title('I attitude (\phi)'); ylabel('I_\phi'); grid on;
subplot(3,3,9); h_Dphi = plot(NaN,NaN,'b-','LineWidth',1.6); title('D attitude (\phi)'); ylabel('D_\phi'); xlabel('Temps (s)'); grid on;

%% ================= FIGURE 3 : Graphiques des erreurs =================
fig_error = figure('Position',[1270 280 600 480],'Color','w','Name','Erreurs (temps reel)');
subplot(3,2,1); h_ex     = plot(NaN,NaN,'r-','LineWidth',1.5); ylabel('e_x (m)');     title('Erreur Position X'); grid on;
subplot(3,2,3); h_ey     = plot(NaN,NaN,'g-','LineWidth',1.5); ylabel('e_y (m)');     title('Erreur Position Y'); grid on;
subplot(3,2,5); h_ez     = plot(NaN,NaN,'b-','LineWidth',1.5); ylabel('e_z (m)');     title('Erreur Altitude Z'); xlabel('Temps (s)'); grid on;
subplot(3,2,2); h_ephi   = plot(NaN,NaN,'r-','LineWidth',1.5); ylabel('e_\phi (rad)'); title('Erreur Roulis (\phi)'); grid on;
subplot(3,2,4); h_etheta = plot(NaN,NaN,'g-','LineWidth',1.5); ylabel('e_\theta (rad)'); title('Erreur Tangage (\theta)'); grid on;
subplot(3,2,6); h_epsi   = plot(NaN,NaN,'b-','LineWidth',1.5); ylabel('e_\psi (rad)'); title('Erreur Lacet (\psi)'); xlabel('Temps (s)'); grid on;

%% Indicateurs de performance en altitude
z_peak = z;
Tr95_z = NaN;

%% ========================= Boucle temps réel =========================
while ishandle(fig_drone) && ishandle(fig_pid) && ishandle(fig_error)

    sd = get(fig_drone,'UserData');   % gains PID actuels (mis à  jour par les sliders)
    t  = t + dt;

    % --- Reset périodique des intégrales (anti-windup) ---
    if t - last_reset >= reset_interval
        int_z=0; int_x=0; int_y=0; int_phi=0; int_theta=0; int_psi=0;
        last_reset = t;
    end

    % --- Génération de rafales de vent aléatoires en continu ---
    wind_x = 0; wind_y = 0;
    if ~wind_active && t >= next_wind_time
        wind_active = true;
        wind_start = t;
        nb_rafales = nb_rafales + 1;
        wind_duration = rand*3 + 1;             % 1 a 4 s
        wind_speed_kmh = rand*8 + 2;            % 2 a 10 km/h
        wind_dir = rand*360;                    % direction alÃ©atoire
        fprintf('\nRafale de vent n°%d (t = %.2f s)\n', nb_rafales, t);
        fprintf('  Intensité : %.1f km/h\n', wind_speed_kmh);
        fprintf('  Durée     : %.1f s\n', wind_duration);
        fprintf('  Direction : %.0f deg\n\n', wind_dir);
        set(h_wind_label,'String',sprintf('Vent : %.1f km/h, Direction : %.0f deg',wind_speed_kmh,wind_dir));
    elseif wind_active && (t - wind_start) >= wind_duration
        wind_active = false;
        set(h_wind_label,'String','Vent : 0.0 km/h, Direction : 0 deg');
        next_wind_time = t + rand*3 + 2;
    end
    if wind_active
        % Coefficient empirique [m/s^2 par km/h]

        % (0.08 - 0.10 m/s^2 pour quelques km/h).
        wind_accel_coeff = 0.015;
        wind_x = wind_accel_coeff*wind_speed_kmh*cosd(wind_dir);
        wind_y = wind_accel_coeff*wind_speed_kmh*sind(wind_dir);
    end

    % --- Boucle Altitude : z_ref -> PID_z -> U1 ---
    ez = z_ref - z;
    int_z = max(min(int_z + ez*dt, int_lim_z), -int_lim_z);
    uz = sd.Kpz*ez + sd.Kiz*int_z - sd.Kdz*vz;
    U1 = m*(G + uz);
    U1 = max(min(U1,U1_max),U1_min);
    Pz = sd.Kpz*ez; Iz_t = sd.Kiz*int_z; Dz_t = -sd.Kdz*vz;

    % --- Boucles externes Position : x,y -> references d'angle ---
    ex = x_ref - x; ey = y_ref - y;

    % --- Mise à  jour des indicateurs de robustesse ---
    if t >= t_eval_start
    max_abs_ex = max(max_abs_ex, abs(ex));
    max_abs_ey = max(max_abs_ey, abs(ey));
    max_abs_ez = max(max_abs_ez, abs(ez));

    z_min = min(z_min, z);
    z_max = max(z_max, z);

        if abs(x) > 1 || abs(y) > 1 || z < z_lim_min || z > z_lim_max
            instabilite = true;
        end
    end

    int_x = max(min(int_x + ex*dt, int_lim_pos), -int_lim_pos);
    int_y = max(min(int_y + ey*dt, int_lim_pos), -int_lim_pos);

    % Commandes des boucles externes de position
    % x'' = G*theta  => theta_ref = PID_x(e_x)
    % y'' = -G*phi   => phi_ref   = -PID_y(e_y)
    theta_ref = sd.Kp_pos*ex + sd.Ki_pos*int_x - sd.Kd_pos*vx;

    pid_y = sd.Kp_pos*ey + sd.Ki_pos*int_y - sd.Kd_pos*vy;
    phi_ref = -pid_y;
    theta_ref = max(min(theta_ref, angle_sat), -angle_sat);
    phi_ref   = max(min(phi_ref,   angle_sat), -angle_sat);
    Px = sd.Kp_pos*ex; Ix_t = sd.Ki_pos*int_x; Dx_t = -sd.Kd_pos*vx;

    % --- Boucles internes Attitude : phi,theta,psi -> couples U2,U3,U4 ---
    ephi   = phi_ref   - phi;
    etheta = theta_ref - theta;
    epsi   = atan2(sin(psi_ref - psi), cos(psi_ref - psi));

    int_phi   = max(min(int_phi   + ephi*dt,   int_lim_ang), -int_lim_ang);
    int_theta = max(min(int_theta + etheta*dt, int_lim_ang), -int_lim_ang);
    int_psi   = max(min(int_psi   + epsi*dt,   int_lim_ang), -int_lim_ang);

    % PID distincts pour les trois axes d'attitude
    U2 = sd.Kp_phi*ephi       + sd.Ki_phi*int_phi       - sd.Kd_phi*p;   % Roll
    U3 = sd.Kp_theta*etheta   + sd.Ki_theta*int_theta   - sd.Kd_theta*q; % Pitch
    U4 = sd.Kp_psi*epsi       + sd.Ki_psi*int_psi       - sd.Kd_psi*r;   % Yaw

    % Termes affiches pour le roll (phi)
    Pphi = sd.Kp_phi*ephi; Iphi_t = sd.Ki_phi*int_phi; Dphi_t = -sd.Kd_phi*p;

    % --- Modele dynamique (lineaire, cf. rapport Partie 1) ---
    zdd     = U1/m - G;
    phidd   = U2/Ix;
    thetadd = U3/Iy;
    psidd   = U4/Iz;
    xdd =  G*theta + wind_x;
    ydd = -G*phi   + wind_y;

    % --- Integration ---
    vz = vz + zdd*dt;     z = z + vz*dt;
    if z < 0, z = 0; vz = 0; end

    p = p + phidd*dt;     phi   = phi   + p*dt;
    q = q + thetadd*dt;   theta = theta + q*dt;
    r = r + psidd*dt;     psi   = atan2(sin(psi + r*dt), cos(psi + r*dt));

    vx = vx + xdd*dt;     x = x + vx*dt;
    vy = vy + ydd*dt;     y = y + vy*dt;

    % Sécurite (uniquement pour l'usage interactif : évite la divergence
    % visuelle si l'utilisateur choisit des gains instables au slider)
    phi   = max(min(phi,   angle_safe), -angle_safe);
    theta = max(min(theta, angle_safe), -angle_safe);

    % --- Mise à jour des buffers (fenêtre glissante T_window) ---
    t_vec = [t_vec, t]; %#ok<AGROW>
    Pz_vec=[Pz_vec,Pz]; Iz_vec=[Iz_vec,Iz_t]; Dz_vec=[Dz_vec,Dz_t]; %#ok<AGROW>
    Px_vec=[Px_vec,Px]; Iy_vec_g=[Iy_vec_g,Ix_t]; Dx_vec=[Dx_vec,Dx_t]; %#ok<AGROW>
    Pphi_vec=[Pphi_vec,Pphi]; Iphi_vec=[Iphi_vec,Iphi_t]; Dphi_vec=[Dphi_vec,Dphi_t]; %#ok<AGROW>
    ex_vec=[ex_vec,ex]; ey_vec=[ey_vec,ey]; ez_vec=[ez_vec,ez]; %#ok<AGROW>
    ephi_vec=[ephi_vec,ephi]; etheta_vec=[etheta_vec,etheta]; epsi_vec=[epsi_vec,epsi]; %#ok<AGROW>

    idx = t_vec >= (t - T_window);
    t_vec = t_vec(idx);
    Pz_vec=Pz_vec(idx); Iz_vec=Iz_vec(idx); Dz_vec=Dz_vec(idx);
    Px_vec=Px_vec(idx); Iy_vec_g=Iy_vec_g(idx); Dx_vec=Dx_vec(idx);
    Pphi_vec=Pphi_vec(idx); Iphi_vec=Iphi_vec(idx); Dphi_vec=Dphi_vec(idx);
    ex_vec=ex_vec(idx); ey_vec=ey_vec(idx); ez_vec=ez_vec(idx);
    ephi_vec=ephi_vec(idx); etheta_vec=etheta_vec(idx); epsi_vec=epsi_vec(idx);

    xtraj=[xtraj,x]; ytraj=[ytraj,y]; ztraj=[ztraj,z]; %#ok<AGROW>
    if numel(xtraj) > numel(t_vec), n = numel(t_vec); xtraj=xtraj(end-n+1:end); ytraj=ytraj(end-n+1:end); ztraj=ztraj(end-n+1:end); end
    set(h_traj,'XData',xtraj,'YData',ytraj,'ZData',ztraj);

    % --- Mise à jour graphiques PID ---
    set(h_Pz,'XData',t_vec,'YData',Pz_vec);   set(h_Iz,'XData',t_vec,'YData',Iz_vec);   set(h_Dz,'XData',t_vec,'YData',Dz_vec);
    set(h_Px,'XData',t_vec,'YData',Px_vec);   set(h_Ix,'XData',t_vec,'YData',Iy_vec_g); set(h_Dx,'XData',t_vec,'YData',Dx_vec);
    set(h_Pphi,'XData',t_vec,'YData',Pphi_vec); set(h_Iphi,'XData',t_vec,'YData',Iphi_vec); set(h_Dphi,'XData',t_vec,'YData',Dphi_vec);
    xlimWin = [max(0,t-T_window), max(t,T_window)];
    hAllPid = [h_Pz h_Iz h_Dz h_Px h_Ix h_Dx h_Pphi h_Iphi h_Dphi];
    for hh = hAllPid, set(get(hh,'Parent'),'XLim',xlimWin); end

    % --- Mise à jour graphiques erreurs ---
    set(h_ex,'XData',t_vec,'YData',ex_vec); set(h_ey,'XData',t_vec,'YData',ey_vec); set(h_ez,'XData',t_vec,'YData',ez_vec);
    set(h_ephi,'XData',t_vec,'YData',ephi_vec); set(h_etheta,'XData',t_vec,'YData',etheta_vec); set(h_epsi,'XData',t_vec,'YData',epsi_vec);
    hAllErr = [h_ex h_ey h_ez h_ephi h_etheta h_epsi];
    for hh = hAllErr, set(get(hh,'Parent'),'XLim',xlimWin); end

    % --- Animation 3D ---
    R = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
         cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
         -sin(theta),          sin(phi)*cos(theta),                            cos(phi)*cos(theta)];

    c = [x;y;z];
    armX1 = c + R*[ Lvis;0;0]; armX2 = c + R*[-Lvis;0;0];
    armY1 = c + R*[0; Lvis;0]; armY2 = c + R*[0;-Lvis;0];
    motors = [armX1 armX2 armY1 armY2];

    set(h_body,'XData',c(1),'YData',c(2),'ZData',c(3));
    set(h_armX,'XData',[armX1(1) armX2(1)],'YData',[armX1(2) armX2(2)],'ZData',[armX1(3) armX2(3)]);
    set(h_armY,'XData',[armY1(1) armY2(1)],'YData',[armY1(2) armY2(2)],'ZData',[armY1(3) armY2(3)]);
    set(h_mot,'XData',motors(1,:),'YData',motors(2,:),'ZData',motors(3,:));

    if wind_active
        u_arrow = 0.3*cosd(wind_dir); v_arrow = 0.3*sind(wind_dir);
        set(h_wind,'XData',c(1),'YData',c(2),'ZData',c(3),'UData',u_arrow,'VData',v_arrow,'WData',0,'Visible','on');
    else
        set(h_wind,'Visible','off');
    end

    % Indicateurs numériques : altitude, erreur, depassement, temps de reponse 95 %
    z_peak = max(z_peak, z);
    if isnan(Tr95_z) && z >= 0.95*z_ref
        Tr95_z = t;
    end
    overshoot_z = max(0, (z_peak - z_ref)/max(z_ref,eps)*100);
    if isnan(Tr95_z)
        tr_txt = '--';
    else
        tr_txt = sprintf('%.2f',Tr95_z);
    end
    set(h_metrics,'String',sprintf('z=%.3f m | e_z=%.3f m | Depassement=%.2f %% | Tr95=%s s', ...
        z, z_ref-z, overshoot_z, tr_txt));

    drawnow limitrate;
end


%% ========================= Bilan de robustesse =========================
fprintf('\n========== BILAN DE ROBUSTESSE ==========\n');
fprintf('Evaluation apres t = %.1f s pour ignorer le decollage initial.\n', t_eval_start);
fprintf('Nombre de rafales : %d\n', nb_rafales);
fprintf('Erreur maximale en x : %.3f m\n', max_abs_ex);
fprintf('Erreur maximale en y : %.3f m\n', max_abs_ey);
fprintf('Erreur maximale en z : %.3f m\n', max_abs_ez);
fprintf('Altitude minimale : %.3f m\n', z_min);
fprintf('Altitude maximale : %.3f m\n', z_max);

if ~instabilite && max_abs_ex <= seuil_xy && max_abs_ey <= seuil_xy && max_abs_ez <= seuil_z
    fprintf('Statut : ROBUSTE - erreurs faibles et aucune divergence detectee\n');
elseif ~instabilite
    fprintf('Statut : STABLE MAIS A AMELIORER - aucune divergence, mais erreurs superieures aux seuils\n');
else
    fprintf('Statut : NON ROBUSTE - instabilite ou sortie de zone detectee\n');
end
fprintf('=========================================\n');


%% ========================= Fonctions locales =========================
function localSliderCallback(fig, fieldName, txtHandle, src)
    sd = get(fig,'UserData');
    sd.(fieldName) = src.Value;
    set(fig,'UserData',sd);
    set(txtHandle,'String',sprintf('%.6g',src.Value));
end


