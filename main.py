from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

import time
import random
import pygame
from pygame.locals import *
from collections import deque

def main():
    x, y = None, None
    window_size = (2150, 2000)
    timestep = 0.05

    mu = 0
    sigma = 50
    noisy = deque(maxlen=50)
    ground = deque(maxlen=50)

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

        fade_trail = pygame.Surface(window_size)
        fade_trail.set_alpha(200)
        fade_trail.fill((0, 0, 0))
        screen.blit(fade_trail, (0, 0))

        if len(noisy) > 1:
            pygame.draw.lines(surface=screen, color=(255,255,0), closed=False, points=list(noisy), width=3)
            pygame.draw.lines(surface=screen, color=(255,0,0), closed=False, points=list(ground), width=3)

        pygame.display.flip()

        print(f"x: {x} | y: {y}")

        time.sleep(timestep - ((time.monotonic() - start_time) % timestep))

    pygame.display.quit()
    pygame.quit()

if __name__ == "__main__":
    main()