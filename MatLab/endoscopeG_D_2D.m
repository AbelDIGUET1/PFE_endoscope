% ------- Endoscope 2D (plan gauche/droite) : MGI + espace de travail -------
clear; close all; clc;
%%% Constantes %%%
L = 12e-3;                 % longueur de la section flexible
theta_max = deg2rad(120);  % limite réelle gauche/droite
%%% Point désiré %%%
theta_p = deg2rad(90);
xp = (L/theta_p) * (1 - cos(theta_p));
zp = (L/theta_p) * sin(theta_p);
P = [xp, zp];
%%% MGI 2D %%%
k = 2*xp / (xp^2 + zp^2);
theta = k*L;
fprintf('k = %.4f m-1 \n', k);
fprintf('theta = kL = %.4f rad \n', theta );
if abs(theta) > theta_max
    fprintf(2,'ATTENTION : point hors espace de travail');
else
    fprintf('Point ATTEIGNABLE \n');
end
%%% Modèle direct le long de la section (s de 0 à L) %%%
s = linspace(0, L, 100);
if k == 0
   x = zeros(size(s));
   z = s;
else
   x = (1/k)*(1-cos(k*s));
   z = (1/k)*sin(k*s);
end
fprintf('Point atteint : [%.4f %.4f]  (attendu : [%.4f %.4f])\n', x(end), z(end), xp, zp);
%% ---------------- Trajectoire d'arrivée ---------------- %%
figure('Name','mouvement');
hold on; grid on; axis equal;
xlabel('x (mm)'); ylabel('z (mm)');
title('Déploiement vers le point cible');
lim = L*1.15*1e3;
xlim([-lim lim]); ylim([-5 lim]);
plot(0,0,'ko','MarkerFaceColor','k');
h_curve = plot(nan,nan,'b-','LineWidth',2);
h_tip   = plot(nan,nan,'ro','MarkerFaceColor','r','MarkerSize',8);
plot(xp*1e3, zp*1e3, 'g^', 'MarkerFaceColor','g', 'MarkerSize',10);
legend('Base','Section','Extrémité','Cible','Location','best');
for i = 1:60
    k_i = k * (i/60);
if k_i == 0
       xs = zeros(size(s));
       zs = s;
else
       xs = (1/k_i)*(1-cos(k_i*s));
       zs = (1/k_i)*sin(k_i*s);
end
    set(h_curve, 'XData', xs*1e3, 'YData', zs*1e3);
    set(h_tip,   'XData', xs(end)*1e3, 'YData', zs(end)*1e3);
    drawnow;
    pause(0.02);
end
%% ---------------- Espace de travail ---------------- %%
figure;
hold on;
grid on;
axis equal;
th = linspace(-pi,pi,400);
th(th==0) = eps;
X = (L./th).*(1-cos(th))*1e3;
Z = (L./th).*sin(th)*1e3;
fill(X,Z,'r','FaceAlpha',0.15,'EdgeColor','none');
plot(X,Z,'r','LineWidth',2);
xlabel('x (mm)')
ylabel('z (mm)')
title('Espace de travail')