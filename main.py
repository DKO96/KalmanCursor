from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

import time
import random
import numpy as np
import pygame
from pygame.locals import *
from collections import deque
from filter import KalmanFilter, LowPassFilter

def main():
    x, y = None, None
    window_size = (2150, 2000)
    timestep = 0.05

    kf = KalmanFilter()
    lp = LowPassFilter(alpha=0.25)
    initialized = False

    mu = 0
    sigma = 50
    noisy = deque(maxlen=50)
    ground = deque(maxlen=50)
    kalman = deque(maxlen=50)
    lowpass = deque(maxlen=50)

    pygame.init()
    screen = pygame.display.set_mode(window_size)
    screen.fill((0, 0, 0))

    start_time = time.monotonic()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                running = False
                break

        gx, gy = pygame.mouse.get_pos()
        x = int(gx + random.gauss(mu, sigma))
        y = int(gy + random.gauss(mu, sigma))

        noisy.append((x, y))
        ground.append((gx, gy))

        y_meas = np.array([x, y]).T

        # Initialize filters
        if not initialized:
            x_init = np.array([x, y, 0, 0])
            P_init = np.eye(4)
            state, cov = kf.kalman_filter(x_init, P_init, y_meas)

            x_prev = x
            y_prev = y
            lx, ly = x_prev, y_prev

            initialized = True
        else:
            # Kalman Filter
            state, cov = kf.kalman_filter(state, cov, y_meas)

            # Low Pass Filter
            prev = [x_prev, y_prev]
            curr = [x, y]
            lx, ly = lp.low_pass_filter(prev, curr)
            x_prev = lx
            y_prev = ly

        kx, ky, vx, vy = state
        kalman.append((kx, ky))
        lowpass.append((lx, ly))

        fade_trail = pygame.Surface(window_size)
        fade_trail.set_alpha(200)
        fade_trail.fill((0, 0, 0))
        screen.blit(fade_trail, (0, 0))

        if len(noisy) > 1:
            pygame.draw.lines(surface=screen, color=(80,80,0), closed=False, points=list(noisy), width=3)
            pygame.draw.lines(surface=screen, color=(75,0,0), closed=False, points=list(ground), width=3)
            # pygame.draw.lines(surface=screen, color=(0,80,0), closed=False, points=list(lowpass), width=3)
            pygame.draw.lines(surface=screen, color=(0,255,255), closed=False, points=list(kalman), width=3)
            # pygame.draw.elliptical

        pygame.display.flip()

        time.sleep(timestep - ((time.monotonic() - start_time) % timestep))

    pygame.display.quit()
    pygame.quit()

if __name__ == "__main__":
    main()