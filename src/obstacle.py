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
        thetas = np.linspace(0, 2*np.pi, self.n) + np.random.uniform(0, 2*np.pi)

        # vertices
        self.verts = np.vstack(([
            # random radius
            np.random.uniform(self.rlb, self.rub)*
            # x and y from theta
            np.array([np.cos(theta), np.sin(theta)]) +
            # position wrt origin
            self.p
            for theta in thetas
        ]))

    def point_inside(self, p):

        # extract point
        x, y = p

        # first vertex
        x0, y0 = self.verts[0, 0], self.verts[0, 1]

        inside = False
        for i in range(self.n + 1):
            x1, y1 = self.verts[i%self.n, 0], self.verts[i%self.n, 1]
            if y > min(y0, y1):
                if y <= max(y0, y1):
                    if x <= max(x0, x1):
                        if y0 != y1:
                            xints = (y-y0)*(x1-x0)/(y1-y0)+x0
                        if x0 == x1 or x <= xints:
                            inside = not inside
            x0, y0 = x1, y1

        return inside

    def line_intersect(self, p):

        # line in question
        a = p[0, :]
        b = p[1, :]

        # first check whether verticies are inside
        if any([self.point_inside(a), self.point_inside(b)]):
            return True

        # vector
        ab = b - a

        # check each edge
        for i in range(self.n - 1):

            # vertex line
            c = self.verts[i, :]
            d = self.verts[i+1, :]

            # vectors
            cd = d - c
            bc = c - b
            bd = d - b
            da = a - d
            db = b - d

            # proper intersction
            if np.cross(ab, bc)*np.cross(ab, bd) < 0 and np.cross(cd, da)*np.cross(cd, db) < 0:
                return True
        return False

    def lines_intersect(self, p):

        for i in range(len(p) - 1):

            line = p[i:i+2, :]
            if self.line_intersect(line):
                return True
        return False

    def plot(self, ax=None):

        if ax is None:
            fig, ax = plt.subplots(1)

        ax.fill(self.verts[:,0], self.verts[:,1], 'gray', alpha=0.6, ec='k')

        return ax


if __name__ == '__main__':

    # instantiate obstacle
    obs = Obstacle(0, 0, 10, 20, 10)

    # plot obstacle
    ax = obs.plot()

    # line
    for i in range(4000):
        line = np.random.uniform(-10, 10, (2,2))
        if obs.lines_intersect(line):
            ax.plot(line[:,0], line[:,1], 'r.-', alpha=0.1)
        else:
            ax.plot(line[:,0], line[:,1], 'g.-')

    ax.set_aspect('equal')
    plt.show()
