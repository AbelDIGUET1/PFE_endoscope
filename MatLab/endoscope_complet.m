% ------- Endoscope COMPLET : tige + section flexible, avec stratégie -------
% Si le point cible n'est pas dans l'espace de travail de la section
% flexible seule, on translate la tige jusqu'à ce qu'il le soit.
clear; close all; clc;

%%% Constantes géométriques (document, p.36) %%%
L_tige = 1.10;             % longueur du corps souple (tige) en m
D_tige = 12e-3;            % diamètre de la tige en m (1.2 cm)

L = 9e-2;                  % longueur de la section flexible orientable (9 cm)
l = 3e-2;                  % longueur du segment rigide portant la caméra (3 cm)
theta_max = deg2rad(120);  % limite angulaire (générique)

%%% Point désiré, exprimé dans le repère GLOBAL (base de la tige) %%%
xp = 0.075;
yp = 0.050;
zp = 0.908;      % volontairement loin -> la section flexible seule ne suffira pas
P_global = [xp, yp, zp];

%%% ---- Étape 1 : recherche de T_tige pour rendre le point atteignable ---- %%%
% On suppose que seule la tige se translate le long de z (insertion/retrait).
% On cherche le T_tige le plus grand possible (insertion max utile) tel que
% le point, une fois exprimé dans R0 (repère base de la section flexible),
% tombe dans l'espace de travail (theta <= theta_max).

T_tige_v = linspace(0, min(zp, L_tige), 2000);  % on ne translate pas au-delà de zp ni L_tige
T_tige = 0;
found = false;

for i = length(T_tige_v):-1:1   % on part du T_tige max et on redescend
    T_test = T_tige_v(i);
    P_local = P_global - [0, 0, T_test];   % point exprimé dans R0
    xl = P_local(1); yl = P_local(2); zl = P_local(3);

    if zl <= 0
        continue   % point derrière la base de la section flexible : impossible
    end

    k_test = (2*sqrt(xl^2+yl^2)) / (xl^2+yl^2+zl^2);
    theta_test = k_test * L;

    if abs(theta_test) <= theta_max
        T_tige = T_test;
        found = true;
        break
    end
end

if ~found
    fprintf(2, 'ATTENTION : aucun T_tige trouvé rendant le point atteignable\n');
    T_tige = 0;
end

fprintf('T_tige choisi = %.1f mm (insertion de la tige)\n', T_tige*1e3);

%%% ---- Étape 2 : MGI de la section flexible dans R0 ---- %%%
P_local = P_global - [0, 0, T_tige];
xl = P_local(1); yl = P_local(2); zl = P_local(3);

Phi = atan2(yl, xl);
k = (2*sqrt(xl^2+yl^2)) / (xl^2+yl^2+zl^2);
theta = k*L;

fprintf('Point dans R0 (apres translation tige) : [%.4f %.4f %.4f]\n', xl, yl, zl);
fprintf('Phi = %.4f rad, k = %.4f m-1, theta = %.1f°\n', Phi, k, rad2deg(theta));

if abs(theta) > theta_max
    fprintf(2, 'Point TOUJOURS hors espace de travail (echec)\n');
else
    fprintf('Point ATTEIGNABLE par la section flexible\n');
end

s = linspace(0, L, 100);
if k == 0
   xf = zeros(size(s)); yf = zeros(size(s)); zf = s;
else
   xf = (1/k)*cos(Phi)*(1-cos(k*s));
   yf = (1/k)*sin(Phi)*(1-cos(k*s));
   zf = (1/k)*sin(k*s);
end
fprintf('Point atteint (R0) : [%.4f %.4f %.4f]  (attendu : [%.4f %.4f %.4f])\n', ...
    xf(end), yf(end), zf(end), xl, yl, zl);

%% ================= FIGURE : Animation en 2 temps ================= %%
figure('Name','endoscope complet - strategie tige+flexible');
hold on; grid on; axis equal;
xlabel('x (mm)'); ylabel('y (mm)'); zlabel('z (mm)');
title('Etape 1: insertion tige -- Etape 2: courbure section flexible');
view(45,25);

lim = max(L_tige*1.05, (L+l)*3) * 1e3;
xlim([-lim*0.15 lim*0.15]); ylim([-lim*0.15 lim*0.15]); zlim([0 lim]);

h_tige   = plot3(nan,nan,nan,'k-','LineWidth',4);
h_curve  = plot3(nan,nan,nan,'b-','LineWidth',2);
h_rigide = plot3(nan,nan,nan,'r-','LineWidth',3);
plot3(P_global(1)*1e3, P_global(2)*1e3, P_global(3)*1e3, 'g^', ...
      'MarkerFaceColor','g', 'MarkerSize',10);
legend('Tige','Section flexible','Segment rigide','Cible','Location','best');

%%% Phase 1 : translation de la tige (k=0, section droite, qui suit la base) %%%
N1 = 50;
for i = 1:N1
    T_i = T_tige * (i/N1);

    z_tige_i = linspace(-L_tige + T_i, T_i, 50);
    x_tige_i = zeros(size(z_tige_i));
    y_tige_i = zeros(size(z_tige_i));
    set(h_tige, 'XData', x_tige_i*1e3, 'YData', y_tige_i*1e3, 'ZData', z_tige_i*1e3);

    % section flexible encore droite (k=0), juste translatee de T_i
    zs_i = s + T_i;
    xs_i = zeros(size(s));
    ys_i = zeros(size(s));
    set(h_curve, 'XData', xs_i*1e3, 'YData', ys_i*1e3, 'ZData', zs_i*1e3);
    set(h_rigide, 'XData', [xs_i(end) xs_i(end)]*1e3, ...
                  'YData', [ys_i(end) ys_i(end)]*1e3, ...
                  'ZData', [zs_i(end) zs_i(end)+l]*1e3);
    drawnow;
    pause(0.01);
end

%%% Phase 2 : courbure de la section flexible (tige fixe a T_tige) %%%
N2 = 60;
for i = 1:N2
    k_i = k * (i/N2);
    if k_i == 0
       xs = zeros(size(s)); ys = zeros(size(s)); zs = s;
    else
       xs = (1/k_i)*cos(Phi)*(1-cos(k_i*s));
       ys = (1/k_i)*sin(Phi)*(1-cos(k_i*s));
       zs = (1/k_i)*sin(k_i*s);
    end
    zs = zs + T_tige;

    if k_i == 0
        tx=0; ty=0; tz=1;
    else
        tx = cos(Phi)*sin(k_i*L);
        ty = sin(Phi)*sin(k_i*L);
        tz = cos(k_i*L);
    end
    t_norm = [tx,ty,tz]/norm([tx,ty,tz]);
    x_rig = [xs(end), xs(end)+t_norm(1)*l];
    y_rig = [ys(end), ys(end)+t_norm(2)*l];
    z_rig = [zs(end), zs(end)+t_norm(3)*l];

    set(h_curve, 'XData', xs*1e3, 'YData', ys*1e3, 'ZData', zs*1e3);
    set(h_rigide, 'XData', x_rig*1e3, 'YData', y_rig*1e3, 'ZData', z_rig*1e3);
    drawnow;
    pause(0.02);
end