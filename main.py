from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

import time
import random
import numpy as np
import pygame
from pygame.locals import *
from collections import deque

class KalmanFilter:
    def __init__(self):
        self.A = np.array([[1, 0, 1, 0],
                           [0, 1, 0, 1],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]
                           ])
        self.C = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]
                           ])
        self.Q = 1e-2 * np.eye(4)
        self.R = 1e1 * np.eye(2)
    
    def kalman_filter(self, x_hat, P_hat, y_meas):
        x_check = self.A @ x_hat
        P_check = self.A @ P_hat @ self.A.T + self.Q

        K = (P_check @ self.C.T)  @ np.linalg.inv(self.C @ P_check @ self.C.T + self.R)

        P_hat = (np.eye(4) - K @ self.C) @ P_check
        x_hat = x_check + K @ (y_meas - self.C @ x_hat)
        
        return x_hat, P_hat

def main():
    x, y = None, None
    window_size = (2150, 2000)
    timestep = 0.05

    kf = KalmanFilter()
    initialized = False

    mu = 0
    sigma = 50
    noisy = deque(maxlen=50)
    ground = deque(maxlen=50)
    estimate = deque(maxlen=50)

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

        if not initialized:
            x_init = np.array([0, 0, 0, 0])
            P_init = np.eye(4)
            state, cov = kf.kalman_filter(x_init, P_init, y_meas)
            initialized = True
        
        state, cov = kf.kalman_filter(state, cov, y_meas)
        ex, ey, vx, vy = state
        estimate.append((ex, ey))

        fade_trail = pygame.Surface(window_size)
        fade_trail.set_alpha(200)
        fade_trail.fill((0, 0, 0))
        screen.blit(fade_trail, (0, 0))

        if len(noisy) > 1:
            pygame.draw.lines(surface=screen, color=(255,255,0), closed=False, points=list(noisy), width=3)
            pygame.draw.lines(surface=screen, color=(255,0,0), closed=False, points=list(ground), width=3)
            pygame.draw.lines(surface=screen, color=(0,0,255), closed=False, points=list(estimate), width=3)

        pygame.display.flip()

        # print(f"x: {x} | y: {y}")

        time.sleep(timestep - ((time.monotonic() - start_time) % timestep))

    pygame.display.quit()
    pygame.quit()

if __name__ == "__main__":
    main()