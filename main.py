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
    trail = deque(maxlen=50)

    pygame.init()
    screen = pygame.display.set_mode(window_size)
    screen.fill((0, 0, 0))
    color = (255, 255, 255)

    start_time = time.monotonic()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                running = False
                break

        raw_x, raw_y = pygame.mouse.get_pos()
        x = int(raw_x + random.gauss(mu, sigma))
        y = int(raw_y + random.gauss(mu, sigma))

        trail.append((x, y))

        fade_surf = pygame.Surface(window_size)
        fade_surf.set_alpha(200)
        fade_surf.fill((0, 0, 0))
        screen.blit(fade_surf, (0, 0))

        if len(trail) > 1:
            pygame.draw.lines(screen, color, False, list(trail), 3)

        pygame.display.flip()

        print(f"x: {x} | y: {y}")

        time.sleep(timestep - ((time.monotonic() - start_time) % timestep))

    pygame.display.quit()
    pygame.quit()

if __name__ == "__main__":
    main()