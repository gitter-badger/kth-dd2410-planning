# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

import numpy as np
from scipy.integrate import ode

class Trajectory(object):

    def __init__(self, dynamics):

        # assign dynamics
        self.dynamics = dynamics
