% ------- Endoscope 3D : MGI + espace de travail (limites asymétriques) -------
clear; close all; clc;

%%% Constantes %%%
L = 12e-3;   % longueur de la section flexible

%%% Limites angulaires %%%
theta_haut = deg2rad(200);   % phi = +pi/2
theta_bas  = deg2rad(100);   % phi = -pi/2 (= 3pi/2)
theta_gd   = deg2rad(120);   % phi = 0 (droite) et phi = pi (gauche)

%%% Point désiré %%%
theta_p = deg2rad(90);     % angle de courbure souhaité
phi_p   = deg2rad(90);     % orientation du plan de courbure (ici : vers le haut)

xp = (L/theta_p) * cos(phi_p) * (1 - cos(theta_p));
yp = (L/theta_p) * sin(phi_p) * (1 - cos(theta_p));
zp = (L/theta_p) * sin(theta_p);
P = [xp, yp, zp];

%%% MGI 3D %%%
Phi = atan2(yp, xp);
k = (2 * sqrt(xp^2 + yp^2)) / (xp^2 + yp^2 + zp^2);
theta = k*L;

fprintf('Phi = %.4f rad \n', Phi);
fprintf('k = %.4f m-1 \n', k);
fprintf('theta = kL = %.4f rad \n', theta);

%%% Limite angulaire correspondant à la direction Phi (interpolation entre
%%% les 4 valeurs cardinales connues : droite(0)-haut(pi/2)-gauche(pi)-bas(3pi/2)) %%%
Phi_mod = mod(Phi, 2*pi);
theta_lim = interp1( ...
    [0, pi/2, pi, 3*pi/2, 2*pi], ...
    [theta_gd, theta_haut, theta_gd, theta_bas, theta_gd], ...
    Phi_mod);

fprintf('theta_lim (direction Phi) = %.4f rad (%.1f°) \n', theta_lim, rad2deg(theta_lim));

if abs(theta) > theta_lim
    fprintf(2,'ATTENTION : point hors espace de travail');
else
    fprintf('Point ATTEIGNABLE \n');
end

%%% Modèle direct le long de la section (s de 0 à L) %%%
s = linspace(0, L, 100);
if k == 0
   x = zeros(size(s));
   y = zeros(size(s));
   z = s;
else
   x = (1/k)*cos(Phi)*(1-cos(k*s));
   y = (1/k)*sin(Phi)*(1-cos(k*s));
   z = (1/k)*sin(k*s);
end
fprintf('Point atteint : [%.4f %.4f %.4f]  (attendu : [%.4f %.4f %.4f])\n', ...
    x(end), y(end), z(end), xp, yp, zp);

%% ================= FIGURE 1 : Trajectoire d'arrivée ================= %%
figure('Name','mouvement');
hold on; grid on; axis equal;
xlabel('x (mm)'); ylabel('y (mm)'); zlabel('z (mm)');
title('Déploiement vers le point cible');
view(45,25);

lim = L*1.15*1e3;
xlim([-lim lim]); ylim([-lim lim]); zlim([-5 lim]);

plot3(0,0,0,'ko','MarkerFaceColor','k');
h_curve = plot3(nan,nan,nan,'b-','LineWidth',2);
h_tip   = plot3(nan,nan,nan,'ro','MarkerFaceColor','r','MarkerSize',8);
plot3(xp*1e3, yp*1e3, zp*1e3, 'g^', 'MarkerFaceColor','g', 'MarkerSize',10);
legend('Base','Section','Extrémité','Cible','Location','best');

for i = 1:60
    k_i = k * (i/60);
    if k_i == 0
       xs = zeros(size(s));
       ys = zeros(size(s));
       zs = s;
    else
       xs = (1/k_i)*cos(Phi)*(1-cos(k_i*s));
       ys = (1/k_i)*sin(Phi)*(1-cos(k_i*s));
       zs = (1/k_i)*sin(k_i*s);
    end
    set(h_curve, 'XData', xs*1e3, 'YData', ys*1e3, 'ZData', zs*1e3);
    set(h_tip,   'XData', xs(end)*1e3, 'YData', ys(end)*1e3, 'ZData', zs(end)*1e3);
    drawnow;
    pause(0.02);
end

%% ================= FIGURE 2 : Espace de travail (limites asymétriques) ================= %%
figure;
hold on;
grid on;
axis equal;

phi = linspace(0,2*pi,100);
Ntheta = 100;

X = zeros(length(phi), Ntheta);
Y = zeros(length(phi), Ntheta);
Z = zeros(length(phi), Ntheta);

for i = 1:length(phi)
    % limite angulaire propre à cette direction phi(i)
    phi_i_mod = mod(phi(i), 2*pi);
    theta_lim_i = interp1([0, pi/2, pi, 3*pi/2, 2*pi], [theta_gd, theta_haut, theta_gd, theta_bas, theta_gd], phi_i_mod);

    th = linspace(eps, theta_lim_i, Ntheta);  % 0 (tendu) jusqu'à la limite réelle

    for j = 1:Ntheta
        X(i,j) = (L/th(j)) * cos(phi(i)) * (1-cos(th(j))) * 1e3;
        Y(i,j) = (L/th(j)) * sin(phi(i)) * (1-cos(th(j))) * 1e3;
        Z(i,j) = (L/th(j)) * sin(th(j)) * 1e3;
    end
end

surf(X,Y,Z,'FaceColor','r','FaceAlpha',0.15,'EdgeColor','none');
plot3(xp*1e3, yp*1e3, zp*1e3, 'g^', 'MarkerFaceColor','g', 'MarkerSize',10);

xlabel('x (mm)')
ylabel('y (mm)')
zlabel('z (mm)')
title('Espace de travail (limites 200°/100°/120°/120°)')
view(45,25)