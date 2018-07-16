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

        # origin and target
        self.origin = self._environment.p0
        self.target = self._environment.pf

        # reset mission
        self.reset()

    def reset(self):

        # state and time record
        self.states = np.empty((0, self._dynamics.sdim), float)
        self.times = np.empty((1, 0), float)

        # reset integrator
        self._integrator = ODE(self._eom, 0, np.hstack((self.origin, np.zeros(self._dynamics.sdim - 2))), 100, jac=self._jac)

    def _eom(self, t, state):
        return self._dynamics.eom_state(state, self._control)

    def _jac(self, t, state):
        return self._dynamics.eom_state_jac(state, self._control)

    def step(self, control):

        # set control
        self._control = control

        # integrate one step
        self._integrator.step()

        # extract state and time
        state, time = self._integrator.y, self._integrator.t

        # check safety
        safe = self._environment.safe(state[:2])

        # check completion
        eps = np.linalg.norm(state[:2] - self.target)
        if eps < 1e-4:
            done = True
        else:
            done = False

        # record state and time
        self.states = np.vstack((self.states, state))
        self.times = np.append(self.times, time)

        # return next state and time
        return state, time, safe, done

    def simulate(self, controls, times=None):

        # reset state and time record
        self.reset()

        # conditions
        safe, done = True, False

        # if given control function
        if callable(controls):

            while safe and not done:

                # get control
                u = controls(self._integrator.t, self._integrator.y)

                # step one time step
                state, time, safe, done = self.step(u)
                print(state)

        elif times is not None and len(controls) == len(times):

            pass

    def plot(self, ax=None):

        if ax is None:
            pass



if __name__ == '__main__':

    # instantiate mission
    mis = Mission()

    # define random controller
    def controller(time, state):
        u =  np.random.uniform(-0.1,0.2)
        return u

    # simulate with that controller
    mis.simulate(controller)

    # plot
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1)
    ax.plot(mis.states[:,0], mis.states[:,1], 'k-')
    mis._environment.plot(ax)
    plt.show()

    mis.reset(None)
