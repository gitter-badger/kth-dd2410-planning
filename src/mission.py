# Christopher Iliffe Sprague
# sprague@kth.se

import numpy as np
from scipy.integrate import LSODA as ODE
from dynamics import Dynamics
from environment import Environment


class Mission(object):

    def __init__(self, dyn=Dynamics(1), env=Environment(50, 30, 1)):

        # dynamics
        self._dynamics = dyn

        # environment
        self._environment = env

        # reset mission
        self.reset()

    def reset(self):

        # state and time record
        self.states = np.empty((0, self._dynamics.sdim), float)
        self.times = np.empty((1, 0), float)

        # safety
        self.safe = True

        # reset state
        self.state = np.hstack((self._environment.p0, np.zeros(self._dynamics.sdim - 2)))

        # reset integrator
        self.integrator = ODE(self._eom, 0, self.state, 10000, jac=self._jac)

    def _eom(self, t, state):
        return self._dynamics.eom_state(state, self.control)

    def _jac(self, t, state):
        return self._dynamics.eom_state_jac(state, self.control)

    def step(self, control):

        # set control
        self.control = control

        # integrate one step
        self.integrator.step()

        # extract state and time
        self.state, self.time = self.integrator.y, self.integrator.t

        # check safety
        if self._environment.safe(self.state[:2]):
            self.safe = True
        else:
            self.safe = False

        # record state and time
        self.states = np.vstack((self.states, self.state))
        self.times = np.append(self.times, self.time)

        # return next state and time
        return self.state, self.time, self.safe
