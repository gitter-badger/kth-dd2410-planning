# Christopher Iliffe Sprague
# sprague@kth.se

import numpy as np, matplotlib.pyplot as plt

class Obstacle(object):

    def __init__(self, x, y, dlb, dub, nvert):

        # cartesian position
        self.p = np.array([x, y], float)

        # radius bounds
        self.rlb, self.rub = dlb/2, dub/2

        # number of vertices
        self.n = nvert

        # generate verticies
        self.gen_verts()

    def gen_verts(self):

        # angles
        thetas = np.linspace(0, 2*np.pi, self.n)

        # random pertubation
        thetas += np.random.uniform(0, 2*np.pi)

        # vertices
        self.verts = np.vstack(([
            np.random.uniform(self.rlb, self.rub)*
            np.array([np.cos(theta), np.sin(theta)]) +
            self.p
            for theta in thetas
        ]))

    def plot(self, ax=None):

        if ax is None:
            fig, ax = plt.subplots(1)

        ax.fill(self.verts[:,0], self.verts[:,1], "gray", ec='k')


if __name__ == '__main__':

    # instantiate obstacle
    obs = Obstacle(0, 0, 10, 20, 10)

    # generate verticies
    obs.gen_verts()
    obs.plot()
