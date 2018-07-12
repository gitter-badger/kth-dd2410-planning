# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

import numpy as np, matplotlib.pyplot as plt
from obstacle import Obstacle

class Environment(object):

    def __init__(self, lx, ly, d):

        # farm dimensions
        self.lx = float(lx)
        self.ly = float(ly)

        # safe radius
        self.d = float(d)

        # random origin and target
        y0 = np.random.uniform(0 + self.d, self.ly - self.d)
        if y0 <= self.ly/2:
            y1 = (y0 + self.ly)/2
        elif y0 > self.ly/2:
            y1 = y0/2
        self.p0 = np.array([0, y0], float)
        self.pf = np.array([self.lx, y1] ,float)

        # generate obstacles
        self.gen_obs()

    def gen_obs(self):

        # obstacle diameter bounds
        dlb, dub = self.d, self.d*4

        # define obstacle area
        xl, xu = self.lx/10 + dub, 9*self.lx/10 - dub
        yl, yu = 0, self.ly

        # obstacle list
        self.obs = list()

        # unsucessfull tries
        j = 0
        # first obstacle
        first = True
        # diameter bounds
        dlb, dub = self.d*2, self.d*6
        # number of vertices per obstacle
        n = 10

        while j < 10000:

            # random position
            x = np.random.uniform(xl, xu)
            y = np.random.uniform(yl, yu)

            # proposed obstacle
            pob = Obstacle(x, y, dlb, dub, n)

            # if first
            if first:
                self.obs.append(pob)
                first = False

            else:

                # check conflictions with other obstacles
                if any([np.linalg.norm(pob.p - sob.p) <= pob.rub + sob.rub + self.d for sob in self.obs]):
                    j += 1
                # check if within boundaries
                elif any(pob.verts[:,0] < 0) or any(pob.verts[:,0] > self.lx) or any(pob.verts[:,1] < 0) or any(pob.verts[:,1] > self.ly):
                    j += 1
                else:
                    self.obs.append(pob)
                    j = 0

    def safe(self, pts):

        # check if insider obstacle
        for ob in self.obs:

            # point
            if pts.ndim == 1:
                if ob.point_inside(pts):
                    return False
                elif pts[0] < 0 or pts[0] > self.lx or pts[1] < 0 or pts[1] > self.ly:
                    return False
                else:
                    continue

            # vector points
            elif pts.ndim == 2:
                if ob.lines_intersect(pts):
                    return False
                elif any(pts[:,0] < 0) or any(pts[:,0] > self.lx) or any(pts[:,1] < 0) or any(pts[:,1] > self.ly):
                    return False
                else:
                    continue

        return True

    def plot(self, ax=None, voronoi=False):

        if ax is None:
            fig, ax = plt.subplots(1)

        # plot walls
        wall = np.array([
            [0, 0],
            [0, self.ly],
            [self.lx, self.ly],
            [self.lx, 0],
            [0, 0]
        ])
        ax.plot(wall[:, 0], wall[:, 1], 'k-', label='Boundaries')
        ax.set_aspect('equal')

        # plot origin and target
        ax.plot(*self.p0, 'ks', label='Origin')
        ax.plot(*self.pf, 'kx', label='Target')

        # plot obstacles
        for i in range(len(self.obs)):
            if i == 0:
                self.obs[i].plot(ax=ax, label=True)
            else:
                self.obs[i].plot(ax=ax)

        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=4, fancybox=True)

        try:
            return fig, ax
        except:
            pass

if __name__ == '__main__':

    env = Environment(50, 30, 1)
    env.plot()
    plt.show()
