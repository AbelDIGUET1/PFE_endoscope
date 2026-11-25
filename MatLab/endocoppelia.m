% ------- Endoscope complet : insertion + rotation + flexion -------
clear; close all; clc;

addpath(genpath('C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\matlab'))

% on ferme une eventuelle connexion encore ouverte avant de relancer
if exist('client', 'var')
    try
        client.delete();
    catch
    end
end

client = RemoteAPIClient();
sim = client.getObject('sim');
sim.setStepping(true);

% lecture de position : chaque composante est convertie en double
% (coppelia renvoie parfois 0 en entier, ce qui bloque cell2mat)
getpos = @(h) cellfun(@double, sim.getObjectPosition(h, -1));

if sim.getSimulationState() == sim.simulation_stopped
    sim.startSimulation();
end

% recuperation des joints
N = 20;
jointFlex = zeros(1,N);
for i = 1:N
    jointFlex(i) = sim.getObject(['/Joint_flex_' num2str(i)]);
end
jointPhi = sim.getObject('/Joint_rotation_phi');
jointInsertion = sim.getObject('/Joint_insertion');
seg19 = sim.getObject('/Segment_flex_19');
seg20 = sim.getObject('/Segment_flex_20');

% constantes du modele
L = 0.09;                  % longueur section flexible = 9cm
len = L / N;               % longueur d'un segment
theta_max = deg2rad(120);  % limite angulaire
L_tige_max = 0.9;          % course max insertion

% on remet tout a 0 avant de commencer
for i = 1:N
    sim.setJointPosition(jointFlex(i), 0);
end
sim.setJointPosition(jointPhi, 0);
sim.setJointPosition(jointInsertion, 0);
sim.step();
pause(0.3);

% position du debut de la section flexible, mesuree en direct (tige au repos)
base0 = getpos(jointFlex(1));
fprintf('base (insertion=0) : x=%.4f y=%.4f z=%.4f\n', base0(1), base0(2), base0(3));

% calibration de phi : on plie de 20 deg avec phi=0 puis phi=90 deg et on
% regarde dans quelle direction (x,z) part le bout de la chaine
ang_cal = deg2rad(20) / N;
angles_cal = ang_cal * ones(1,N);
angles_cal(1) = ang_cal / 2;
phi_cal = [0, deg2rad(90)];
b = zeros(2,2);

for t = 1:2
    for i = 1:N
        sim.setJointPosition(jointFlex(i), 0);
    end
    sim.setJointPosition(jointPhi, phi_cal(t));
    for i = 1:N
        sim.setJointPosition(jointFlex(i), angles_cal(i));
    end
    sim.step();
    pause(0.3);
    p20 = getpos(jointFlex(N));
    v = [p20(1)-base0(1), p20(3)-base0(3)];
    b(t,:) = v / norm(v);
end
fprintf('direction phi=0  : x=%.3f z=%.3f\n', b(1,1), b(1,2));
fprintf('direction phi=90 : x=%.3f z=%.3f\n', b(2,1), b(2,2));

for i = 1:N
    sim.setJointPosition(jointFlex(i), 0);
end
sim.setJointPosition(jointPhi, 0);
sim.step();
pause(0.3);

% point qu'on veut atteindre : on donne l'ecart lateral par rapport a l'axe
% de la tige (la tige est en z=0.075, pas en z=0)
yp = -0.80;
dx_lat = 0.03;
dz_lat = 0.02;
xp = base0(1) + dx_lat;
zp = base0(3) + dz_lat;
P_global = [xp, yp, zp];

% on affiche un petit repere pour voir la cible dans la scene
try
    oldTarget = sim.getObject('/Cible_visuelle');
    sim.removeObject(oldTarget);
catch
end
targetHandle = sim.createDummy(0.01);
sim.setObjectAlias(targetHandle, 'Cible_visuelle');
sim.setObjectPosition(targetHandle, -1, {xp, yp, zp});
sim.setObjectColor(targetHandle, 0, sim.colorcomponent_ambient_diffuse, [0, 1, 0]);

% ecart lateral entre l'axe de la tige et le point
lat = [xp - base0(1), zp - base0(3)];
r = norm(lat);

% la chaine a une longueur fixe L : pour un ecart lateral r il n'y a qu'une
% seule distance axiale a possible (celle ou l'arc fait exactement L)
%   atan2(r,a) * (r^2 + a^2) / r = L      (resolu par bisection)
if r < 1e-6
    a_star = L;
    ok = true;
else
    a_lo = r / tan(theta_max/2);
    a_hi = L;
    g = @(a) atan2(r,a) * (r^2 + a^2) / r - L;
    ok = (g(a_lo) * g(a_hi) <= 0);
    if ok
        for it = 1:80
            a_m = (a_lo + a_hi) / 2;
            if g(a_lo) * g(a_m) <= 0
                a_hi = a_m;
            else
                a_lo = a_m;
            end
        end
        a_star = (a_lo + a_hi) / 2;
    end
end

if ~ok
    fprintf(2, 'point hors espace de travail : ecart lateral %.1f mm trop grand (max environ 64 mm)\n', r*1e3);
    sim.setStepping(false);
    clear client sim
    return
end

% la tige s'insere jusqu'a ce qu'il reste a_star entre la base et la cible (en y)
T_tige = (base0(2) - yp) - a_star;

if T_tige < 0 || T_tige > L_tige_max
    fprintf(2, 'point hors espace de travail : insertion de %.1f mm hors course\n', T_tige*1e3);
    sim.setStepping(false);
    clear client sim
    return
end

% angle de courbure total et orientation du plan de courbure
if r < 1e-6
    theta = 0;
    Phi = 0;
else
    theta = 2 * atan2(r, a_star);
    t_dir = lat / r;
    Phi = atan2(t_dir * b(2,:)', t_dir * b(1,:)');
end

fprintf('ecart lateral r = %.1f mm\n', r*1e3);
fprintf('a_star = %.2f mm\n', a_star*1e3);
fprintf('T_tige = %.1f mm\n', T_tige*1e3);
fprintf('Phi = %.4f rad (%.1f deg)\n', Phi, rad2deg(Phi));
fprintf('theta = %.4f rad (%.1f deg)\n', theta, rad2deg(theta));
fprintf('point atteignable\n');

% angles des joints : premier a ang/2, les autres a ang
ang = theta / N;
angles = ang * ones(1,N);
angles(1) = ang / 2;

% phase 1 : on insere la tige petit a petit
N_steps = 20;
for step = 1:N_steps
    sim.setJointPosition(jointInsertion, T_tige * (step/N_steps));
    pause(0.03);
    sim.step();
end
T_reel = sim.getJointPosition(jointInsertion);
fprintf('T_tige REEL apres phase 1 = %.4f m (demande = %.4f)\n', T_reel, T_tige);

% phase 2 : on tourne phi
for step = 1:N_steps
    sim.setJointPosition(jointPhi, Phi * (step/N_steps));
    pause(0.03);
    sim.step();
end

% phase 3 : la section flexible se courbe
for step = 1:N_steps
    for i = 1:N
        sim.setJointPosition(jointFlex(i), angles(i) * (step/N_steps));
    end
    pause(0.03);
    sim.step();
end
pause(0.3);

fprintf('deploiement termine\n');
fprintf('T_tige=%.1fmm, Phi=%.1fdeg, theta=%.1fdeg\n', T_tige*1e3, rad2deg(Phi), rad2deg(theta));

% verification : bout de la chaine = centre du segment 20 + demi-segment
c19 = getpos(seg19);
c20 = getpos(seg20);
u = (c20 - c19) / norm(c20 - c19);
tip = c20 + u * len / 2;
erreur = norm(tip - P_global);
fprintf('bout du robot : x=%.4f y=%.4f z=%.4f\n', tip(1), tip(2), tip(3));
fprintf('cible         : x=%.4f y=%.4f z=%.4f\n', xp, yp, zp);
fprintf('erreur = %.2f mm\n', erreur*1e3);

sim.setStepping(false);
pause(0.2);
clear client sim