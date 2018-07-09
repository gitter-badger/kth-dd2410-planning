# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

import numpy as np, matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
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

        # paths
        self.path0, self.path1 = self.gen_paths()

        # generate obstacles
        self.gen_obs()

    def gen_paths(self):

        # horizontal point
        x = self.lx/2

        # random y
        y = np.random.uniform(0 + self.d, self.ly - self.d)

        # first path
        path0 = np.vstack((self.p0, [x, y], self.pf))

        # another y
        if y <= self.ly/2:
            y = (y + self.ly)/2
        elif y > self.ly/2:
            y = y/2

        # second path
        path1 = np.vstack((self.p0, [x, y], self.pf))

        # interpolate
        path0 = CubicSpline(path0[:,0], path0[:,1], bc_type='clamped')
        path1 = CubicSpline(path1[:,0], path1[:,1], bc_type='clamped')

        return path0, path1

    def gen_obs(self):

        # obstacle diameter bounds
        dlb, dub = self.d/2, self.d*4

        # define obstacle area
        xl, xu = self.lx/10 + dub, 9*self.lx/10 - dub
        yl, yu = 0, self.ly

        # obstacle list
        self.obs = list()

        # number of obstacles
        n = 40
        # succesfull obstacles
        i = 0
        # unsucessfull tries
        j = 0
        # first obstacle
        first = True

        while j < 1000:

            # random position
            x = np.random.uniform(xl, xu)
            y = np.random.uniform(yl, yu)

            # diameter bounds
            dlb, dub = self.d, self.d*2

            # proposed obstacle
            pob = Obstacle(x, y, dlb, dub, 8)

            # if first
            if first:
                self.obs.append(pob)
                first = False

            else:

                # check conflictions with other obstacles
                if any([np.linalg.norm(pob.p - sob.p) <= pob.rub + sob.rub + self.d for sob in self.obs]):
                    j += 1
                    continue

                # check if within boundaries
                elif any([any([vert[0] < 0, vert[0] > self.lx, vert[1] < 0, vert[1] > self.ly]) for vert in pob.verts]):
                    j += 1
                    continue

                else:
                    self.obs.append(pob)
                    i += 1
                    j = 0

    def safe(self, p):

        # check if insider obstacle
        for ob in self.obs:
            if ob.inside(p):
                return False
            else:
                continue
        return True


    def plot(self, ax=None):

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
        ax.plot(wall[:, 0], wall[:, 1], 'k-', label='Wall')
        ax.set_aspect('equal')

        # plot origin and target
        ax.plot(*self.p0, 'kx', label='Origin')
        ax.plot(*self.pf, 'kx')

        # plot paths
        #x = np.linspace(0, self.lx, 100*self.lx)
        #ax.plot(x, self.path0(x))
        #ax.plot(x, self.path1(x))

        for obs in self.obs:
            obs.plot(ax)




if __name__ == '__main__':

    # instantiate environment
    env = Environment(50, 30, 1)
    env.gen_obs()

    fig, ax = plt.subplots(1)
    env.plot(ax)

    # random points
    i = 0
    while i < 1000:
        x = np.random.uniform(0, env.lx)
        y = np.random.uniform(0, env.ly)
        p = np.array([x, y])
        if env.safe(p):
            ax.plot(*p, 'k.')
            i += 1
        else:
            continue

    #[ax.plot(p[:,0], p[:,1]) for p in pos]
    #[ax.plot(pn[:,0], pn[:,1], 'k--') for pn in bordn]
    #[ax.plot(ps[:,0], ps[:,1], 'k--') for ps in bords]
    #[ax.quiver(p[:, 0], p[:, 1], norm[:,0], norm[:,1], scale=None) for p,norm in zip(pos, norms)]

    plt.show()
