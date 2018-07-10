# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

import numpy as np, matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from obstacle import Obstacle
from scipy.spatial import Voronoi

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

        # generate voronoi graph
        self.gen_voronoi()

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
            dlb, dub = self.d*2, self.d*5

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

    def safe(self, pts):

        # check if insider obstacle
        for ob in self.obs:

            # point
            if pts.ndim == 1:
                if ob.point_inside(pts):
                    return False
                else:
                    continue

            # vector points
            elif pts.ndim == 2:
                if ob.lines_intersect(pts):
                    return False
                else:
                    continue

        return True

    def gen_voronoi(self):

        verts = np.vstack(([ob.verts for ob in self.obs]))
        voronoi = Voronoi(verts).vertices
        gv = list()
        for vp in voronoi:
            x, y = vp
            if x >= 0 and x <= self.lx and y >= 0 and y <= self.ly and self.safe(vp):
                gv.append(vp)
        self.voronoi = np.array(gv)



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

        # plot obstacles
        for obs in self.obs:
            obs.plot(ax)

        # plot voronoi graph
        ax.plot(self.voronoi[:,0], self.voronoi[:,1], 'k.')







if __name__ == '__main__':

    # instantiate environment
    env = Environment(50, 30, 1)
    #env.gen_obs()

    fig, ax = plt.subplots(1)
    env.plot(ax)


    # random points
    '''
    i = 0
    while i < 1:
        x = np.random.uniform(0, env.lx, 3)
        y = np.random.uniform(0, env.ly, 3)
        p = np.vstack((x, y)).T
        p = np.vstack((env.p0, p, env.pf))
        #p = np.array([x, y])
        if env.safe(p):
            ax.plot(p[:,0], p[:,1], 'k.-')
            i += 1
            print(p)
        else:
            ax.plot(x, y, 'k.-', alpha=0.1)
            continue

    #[ax.plot(p[:,0], p[:,1]) for p in pos]
    #[ax.plot(pn[:,0], pn[:,1], 'k--') for pn in bordn]
    #[ax.plot(ps[:,0], ps[:,1], 'k--') for ps in bords]
    #[ax.quiver(p[:, 0], p[:, 1], norm[:,0], norm[:,1], scale=None) for p,norm in zip(pos, norms)]
    '''

    plt.show()
    print(env.p0)
