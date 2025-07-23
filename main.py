from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

import random
import numpy as np
import pygame
from pygame.locals import *
from collections import deque
from filter import KalmanFilter, LowPassFilter
from shapes import Shapes

def main():
    x, y = None, None
    window_size = (2150, 2000)
    pygame.init()
    pygame.display.set_caption("Kalman Filter Mouse Estimator")
    screen = pygame.display.set_mode(window_size)
    screen.fill((0, 0, 0))

    timestep = 0.01
    kf = KalmanFilter(timestep)
    lp = LowPassFilter(alpha=0.1)
    shapes = Shapes(window_size)
    initialized = False

    sigma = 50
    maxlen = 150    # modify length of trailing edge
    noisy = deque(maxlen=maxlen)
    ground = deque(maxlen=maxlen)
    filter = deque(maxlen=maxlen)

    # waypoints = shapes.circle(750, 'cw', N=300)
    waypoints = shapes.figure8(1200, 1000, 'cw', N=400)
    i = 0

    running = True
    clock = pygame.time.Clock()
    while running:
        dt_ms = clock.tick(1/timestep)

        if i == len(waypoints):
            i = 0

        for event in pygame.event.get():
            if event.type == KEYDOWN:
                running = False
                break
        
        gx, gy = waypoints[i]
        i += 1

        x = int(gx + random.gauss(0, sigma))
        y = int(gy + random.gauss(0, sigma))

        noisy.append((x, y))
        ground.append((gx, gy))

        y_meas = np.array([x, y]).T

        # Initialize filters
        if not initialized:
            x_init = np.array([x, y, 0, 0])
            P_init = np.eye(4)
            state, cov = kf.kalman_filter(x_init, P_init, y_meas)

            kx, ky, vx, vy = state

            x_prev = kx
            y_prev = ky
            lx, ly = x_prev, y_prev

            initialized = True
        else:
            # Kalman Filter
            state, cov = kf.kalman_filter(state, cov, y_meas)
            kx, ky, vx, vy = state

            # Low Pass Filter
            prev = [x_prev, y_prev]
            curr = [kx, ky]
            lx, ly = lp.low_pass_filter(prev, curr)
            x_prev = lx
            y_prev = ly

        filter.append((lx, ly))

        fade_trail = pygame.Surface(window_size)
        fade_trail.set_alpha(200)
        fade_trail.fill((0, 0, 0))
        screen.blit(fade_trail, (0, 0))

        if len(noisy) > 1:
            pygame.draw.lines(surface=screen, color=(100,100,0), closed=False, points=list(noisy), width=3)
            pygame.draw.lines(surface=screen, color=(255,0,0), closed=False, points=list(ground), width=5)
            pygame.draw.lines(surface=screen, color=(0,255,255), closed=False, points=list(filter), width=5)

        pygame.display.flip()

    pygame.display.quit()
    pygame.quit()

if __name__ == "__main__":
    main()