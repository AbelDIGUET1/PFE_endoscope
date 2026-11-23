"""Pilotage articulaire de l'endoscope dans CoppeliaSim, au clavier.

Fleches gauche/droite : insertion de la tige
Fleches haut/bas      : rotation phi
Q / D                 : flexion theta de la partie flexible
R                     : remise a zero | Echap : quitter
"""
import math
import time

import pygame
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

#parametres
N = 20                            # segments de la section flexible
T_MIN, T_MAX = 0.0, 0.9           # course d'insertion (m)
THETA_MIN = math.radians(-120)    # courbure totale min (rad)
THETA_MAX = math.radians(120)     # courbure totale max (rad)
# le doc donne 200 deg (haut), 100 deg (bas), 120 deg (cotes) : a ajuster ici plus tard

V_INSERTION = 0.05                # m/s
V_ROTATION = math.radians(60)     # rad/s
V_FLEXION = math.radians(40)      # rad/s
FPS = 30


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def main():
    # connexion a CoppeliaSim
    client = RemoteAPIClient()
    sim = client.require('sim')
    client.setStepping(True)
    if sim.getSimulationState() == sim.simulation_stopped:
        sim.startSimulation()

    joint_flex = [sim.getObject(f'/Joint_flex_{i}') for i in range(1, N + 1)]
    joint_phi = sim.getObject('/Joint_rotation_phi')
    joint_ins = sim.getObject('/Joint_insertion')
    cam = sim.getObject('/Segment_camera', {'noError': True})   # -1 si absente

    def send(T, phi, theta):
        ang = theta / N
        sim.setJointPosition(joint_ins, T)
        sim.setJointPosition(joint_phi, phi)
        for i, h in enumerate(joint_flex):
            sim.setJointPosition(h, ang / 2 if i == 0 else ang)

    # fenetre clavier
    pygame.init()
    screen = pygame.display.set_mode((700, 270))
    pygame.display.set_caption('Pilotage articulaire endoscope (garder le focus ici)')
    font = pygame.font.SysFont('consolas', 18)
    clock = pygame.time.Clock()

    T, phi, theta = 0.0, 0.0, 0.0
    send(T, phi, theta)
    client.step()
    last_sent = (T, phi, theta)

    cam_pos = None
    t_cam = 0.0
    t_prev = time.time()
    running = True

    try:
        while running:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False

            keys = pygame.key.get_pressed()
            if keys[pygame.K_ESCAPE]:
                running = False

            now = time.time()
            dt = min(now - t_prev, 0.1)
            t_prev = now
            slow = 0.25 if (pygame.key.get_mods() & pygame.KMOD_SHIFT) else 1.0

            # insertion : droite = inserer, gauche = retirer
            T += (keys[pygame.K_RIGHT] - keys[pygame.K_LEFT]) * V_INSERTION * dt * slow
            # rotation : haut = +, bas = -  (on reste dans ]-pi, pi])
            phi += (keys[pygame.K_UP] - keys[pygame.K_DOWN]) * V_ROTATION * dt * slow
            phi = (phi + math.pi) % (2 * math.pi) - math.pi
            # flexion : D = +, Q = -
            theta += (keys[pygame.K_d] - keys[pygame.K_q]) * V_FLEXION * dt * slow

            if keys[pygame.K_r]:
                T, phi, theta = 0.0, 0.0, 0.0

            T = clamp(T, T_MIN, T_MAX)
            theta = clamp(theta, THETA_MIN, THETA_MAX)

            # on n'envoie que si q a change (chaque envoi = 22 appels reseau)
            if (T, phi, theta) != last_sent:
                send(T, phi, theta)
                last_sent = (T, phi, theta)
            client.step()

            # lecture de la camera, quelques fois par seconde seulement
            if cam != -1 and now - t_cam > 0.25:
                cam_pos = sim.getObjectPosition(cam, -1)
                t_cam = now

            # affichage
            lines = [
                f'insertion   T     = {T * 1000:8.1f} mm',
                f'rotation    phi   = {math.degrees(phi):8.1f} deg',
                f'flexion     theta = {math.degrees(theta):8.1f} deg',
                '',
            ]
            if cam_pos is not None:
                lines.append(f'camera (centre) x={cam_pos[0]:.4f}  y={cam_pos[1]:.4f}  z={cam_pos[2]:.4f}')
            else:
                lines.append('camera (centre) : /Segment_camera introuvable')
            lines += [
                '',
                'gauche/droite : insertion | haut/bas : rotation | Q/D : flexion',
                'Maj : lent | R : zero | Echap : quitter',
            ]
            screen.fill((25, 25, 30))
            for k, line in enumerate(lines):
                screen.blit(font.render(line, True, (230, 230, 230)), (15, 15 + 28 * k))
            pygame.display.flip()
            clock.tick(FPS)

    finally:
        client.setStepping(False)
        pygame.quit()


if __name__ == '__main__':
    main()