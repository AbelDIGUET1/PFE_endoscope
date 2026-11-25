% ------- Endoscope 2D piloté sur CoppeliaSim via MGI -------
clear; close all; clc;

if exist('client', 'var')
    try
        client.delete();
    catch
    end
end

client = RemoteAPIClient();
sim = client.getObject('sim');
sim.setStepping(true);

if sim.getSimulationState() == sim.simulation_stopped
    sim.startSimulation();
end

N = 20;
jointFlex = zeros(1,N);
for i = 1:N
    jointFlex(i) = sim.getObject(['/Joint_flex_' num2str(i)]);
end

%%% Constantes %%%
L = 0.09;
theta_max = deg2rad(120);

%%% Point désiré %%%
theta_p = deg2rad(90);
xp = (L/theta_p) * (1 - cos(theta_p));
zp = (L/theta_p) * sin(theta_p);

%%% MGI 2D %%%
k = 2*xp / (xp^2 + zp^2);
theta = k*L;
fprintf('k = %.4f m-1 \n', k);
fprintf('theta = kL = %.4f rad \n', theta);

if abs(theta) > theta_max
    fprintf(2,'ATTENTION : point hors espace de travail\n');
else
    fprintf('Point ATTEIGNABLE \n');
end

%%% Reset à 0 %%%
for i = 1:N
    sim.setJointPosition(jointFlex(i), 0);
end
sim.step();

%%% k_i varie de 0 à k, réparti sur les 20 joints %%%
N_steps = 30;   
for step = 1:N_steps
    k_i = k * (step/N_steps);
    theta_i = k_i * L;
    angle_par_joint = theta_i / N;

    for i = 1:N
        sim.setJointPosition(jointFlex(i), angle_par_joint);
    end
    sim.step();
end

fprintf('Deploiement termine. theta final = %.4f rad (%.1f deg)\n', theta, rad2deg(theta));

%%% Fermeture propre %%%
% sim.stopSimulation();   % décommente si tu veux arrêter la simu à la fin
% client.delete();        % décommente pour fermer proprement la connexion