# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

import numpy as np, matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

class Environment(object):

    def __init__(self, lx, ly, r):

        # farm dimensions
        self.lx = float(lx)
        self.ly = float(ly)

        # safe radius
        self.r = float(r)

        # random origin and target
        self.p0 = np.array([0, np.random.uniform(0 + self.r, self.ly - self.r)], float)
        self.pf = np.array([self.lx, np.random.uniform(0 + self.r, self.ly - self.r)] ,float)

    def gen_paths(self, nx=4, npaths=2):

        # horizontal boundaries
        x = np.linspace(0, self.lx, nx+3)[1:-1]

        paths = list()
        for i in range(npaths):
            path = [self.p0]
            for j in range(nx)[1:]:
                path.append([
                    np.random.uniform(x[i-1], x[i]),
                    np.random.uniform(self.r, self.ly - self.r)
                ])
            paths.append(np.vstack((path, self.pf)))

        fig, ax = plt.subplots(1)
        [ax.plot(p[:,0], p[:,1]) for p in paths]
        plt.show()



        '''
        # first path
        path0 = np.vstack((
            self.p0,
            [np.random.uniform(self.lx/4, 2*self.lx/4)]
        ))



        wps = np.hstack((
            np.random.uniform(self.r, self.lx - self.r, size=(2,1)),
            np.random.uniform(self.r, self.ly - self.r, size=(2,1))
        ))

        # paths
        paths = [np.vstack((self.p0, wp, self.pf)) for wp in wps]

        print(paths)

        # cubic splines
        y = [CubicSpline(path[:, 0], path[:, 1], bc_type='clamped') for path in paths]

        # derrivatives
        dy = [cs.derivative() for cs in y]

        # x points
        npts = 100
        x, dx = np.linspace(0, self.lx, npts, retstep=True)

        # normals
        norms = [np.vstack((-der(x), np.full(npts, dx))).T for der in dy]
        norms = [norm/np.linalg.norm(norm, axis=1)[:, None] for norm in norms]

        # positions
        pos = [np.vstack((x, cs(x))).T for cs in y]

        # pertubations
        pertn = [norm*self.r*1.4 for norm in norms]
        perts = [norm*-self.r*1.4 for norm in norms]

        # borders
        bordn = [p + pn for p,pn in zip(pos, pertn)]
        bords = [p + ps for p,ps in zip(pos, perts)]

        return pos, norms, bordn, bords
        '''

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



if __name__ == '__main__':

    # instantiate environment
    env = Environment(50, 30, 1)

    # plot
    env.gen_paths()

    #fig, ax = plt.subplots(1)
    #env.plot(ax)

    #[ax.plot(p[:,0], p[:,1]) for p in pos]
    #[ax.plot(pn[:,0], pn[:,1], 'k--') for pn in bordn]
    #[ax.plot(ps[:,0], ps[:,1], 'k--') for ps in bords]
    #[ax.quiver(p[:, 0], p[:, 1], norm[:,0], norm[:,1], scale=None) for p,norm in zip(pos, norms)]

    #plt.show()
