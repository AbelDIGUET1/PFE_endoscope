"""Pilotage articulaire de l'endoscope dans CoppeliaSim, a la manette.

Croix directionnelle gauche/droite : insertion de la tige
Stick gauche (vertical)            : rotation phi
Stick droit (horizontal)           : flexion theta
Start : remise a zero | Back : quitter

"""
import math
import os
import time

os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'

import pygame
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# parametres 
N = 20                            # segments de la section flexible
T_MIN, T_MAX = 0.0, 0.9           # course d'insertion (m)
THETA_MIN = math.radians(-180)    # courbure totale min (rad)
THETA_MAX = math.radians(180)     # courbure totale max (rad)

V_INSERTION = - 0.05                # m/s a fond de stick
V_ROTATION = math.radians(60)     # rad/s a fond de stick
V_FLEXION = - math.radians(40)      # rad/s a fond de stick
FPS = 30

# affectation de la manette
CROIX = 0                         # croix directionnelle (hat 0) : gauche/droite = insertion
AXE_ROTATION = 1                  # stick gauche, vertical (haut = valeur negative)
AXE_FLEXION = 2                   # stick droit, horizontal
BOUTON_RESET = 7                  # Start
BOUTON_QUITTER = 6                # Back
ZONE_MORTE = 0.15


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def axe(joy, i):
    v = joy.get_axis(i)
    if abs(v) < ZONE_MORTE:
        return 0.0
    return math.copysign((abs(v) - ZONE_MORTE) / (1 - ZONE_MORTE), v)


def main():
    # manette 
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print('Aucune manette detectee : branche-la puis relance le script.')
        pygame.quit()
        return
    joy = pygame.joystick.Joystick(0)
    print(f'manette : {joy.get_name()} ({joy.get_numaxes()} axes, {joy.get_numbuttons()} boutons, '
          f'{joy.get_numhats()} croix)')
    if joy.get_numhats() <= CROIX:
        print('attention : croix directionnelle non detectee comme hat, l\'insertion ne marchera pas')

    #connexion a CoppeliaSim
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
        """Envoie q = (T, phi, theta) aux joints. Le premier joint de flexion prend la
        moitie de l'angle des autres (regle du milieu), comme dans les scripts MATLAB."""
        ang = theta / N
        sim.setJointPosition(joint_ins, T)
        sim.setJointPosition(joint_phi, phi)
        for i, h in enumerate(joint_flex):
            sim.setJointPosition(h, ang / 2 if i == 0 else ang)

    #fenetre d'affichage
    screen = pygame.display.set_mode((700, 250))
    pygame.display.set_caption('Pilotage articulaire endoscope (manette)')
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

            if joy.get_button(BOUTON_QUITTER):
                running = False

            now = time.time()
            dt = min(now - t_prev, 0.1)
            t_prev = now

            # insertion : croix vers la droite = inserer, vers la gauche = retirer
            croix_x = joy.get_hat(CROIX)[0] if joy.get_numhats() > CROIX else 0
            T += croix_x * V_INSERTION * dt
            # rotation : stick vers le haut = +, vers le bas = - (on reste dans ]-pi, pi])
            phi += -axe(joy, AXE_ROTATION) * V_ROTATION * dt
            phi = (phi + math.pi) % (2 * math.pi) - math.pi
            # flexion : droite = +, gauche = -
            theta += axe(joy, AXE_FLEXION) * V_FLEXION * dt

            if joy.get_button(BOUTON_RESET):
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
            lines += ['', 'croix gauche/droite : insertion | stick gauche : rotation | stick droit : flexion',
                  'Start : zero | Back : quitter']
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