import numpy as np

class Shapes:
    def __init__(self, center):
        self.center_x = center[0] // 2
        self.center_y = center[1] // 2

    def circle(self, rad, dir, N=300, start=0.0, end=2*np.pi):
        if dir == 'cw':
            end = -end

        angles = np.linspace(start, start+end, N, endpoint=True)
        x = self.center_x + rad * np.cos(angles)
        y = self.center_y + rad * np.sin(angles)

        return list(zip(x,y))

    def figure8(self, width, height, dir, N=500):
        sweep = -2 * np.pi if dir == 'cw' else 2 * np.pi
        t = np.linspace(0, 0 + sweep, N, endpoint=True)

        a = width / 2.0
        b = height / 2.0

        denom = np.sin(t)**2 + 1.0
        x = self.center_x + (a * np.sqrt(2) * np.cos(t)) / denom
        y = self.center_y + (b * np.sqrt(2) * np.cos(t) * np.sin(t)) / denom

        return list(zip(x,y))

