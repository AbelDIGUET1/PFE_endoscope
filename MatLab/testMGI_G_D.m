clear; close all; clc;

%% Paramètres

L = 12e-3;
theta_max = deg2rad(120);

%% Point cible

theta_p = deg2rad(90);

xp = (L/theta_p)*(1-cos(theta_p));
zp = (L/theta_p)*sin(theta_p);

%% MGI

k = 2*xp/(xp^2 + zp^2);
theta = k*L;

fprintf('k = %.4f m^-1\n',k);
fprintf('theta = %.2f deg\n',rad2deg(theta));

%% Modèle direct

s = linspace(0,L,100);

if abs(k) < 1e-9
    x = zeros(size(s));
    z = s;
else
    x = (1/k)*(1-cos(k*s));
    z = (1/k)*sin(k*s);
end

fprintf('Point atteint : [%.4f %.4f]\n',x(end),z(end));

%% Déploiement

figure;
hold on;
grid on;
axis equal;

xlabel('x (mm)');
ylabel('z (mm)');

for i = 1:60

    k_i = k*i/60;

    if abs(k_i) < 1e-9
        xs = zeros(size(s));
        zs = s;
    else
        xs = (1/k_i)*(1-cos(k_i*s));
        zs = (1/k_i)*sin(k_i*s);
    end

    cla
    plot(xs*1e3,zs*1e3,'b','LineWidth',2)
    plot(xs(end)*1e3,zs(end)*1e3,'ro','MarkerFaceColor','r')

    axis equal
    grid on

    drawnow
    pause(0.02)

end

%% Espace de travail

figure;
hold on;
axis equal;

th = linspace(-theta_max,theta_max,400);

th(abs(th)<1e-6) = 1e-6;

X = (L./th).*(1-cos(th))*1e3;
Z = (L./th).*sin(th)*1e3;

fill(X,Z,'r','FaceAlpha',0.2,'EdgeColor','none');
plot(X,Z,'r','LineWidth',2);

xlabel('x (mm)');
ylabel('z (mm)');
title('Espace de travail');