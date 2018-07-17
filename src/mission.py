# Christopher Iliffe Sprague
# sprague@kth.se

import numpy as np
from scipy.integrate import LSODA as ODE
from dynamics import Dynamics
from environment import Environment
import matplotlib.pyplot as plt


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
        self.states = np.array([np.hstack((self.origin, np.zeros(self._dynamics.sdim - 2)))])
        self.times = np.array([0])

        # reset integrator
        self._integrator = ODE(self._eom, 0, self.states[0], 10000, jac=self._jac, atol=1e-10)

    def _eom(self, t, state):
        return self._dynamics.eom_state(state, self._control)

    def _jac(self, t, state):
        return self._dynamics.eom_state_jac(state, self._control)

    def safe(self, state):
        return self._environment.safe(state[:2])

    def done(self, state):
        delta = np.linalg.norm(state[:2] - self.target)
        if delta < 1e-4:
            return True
        else:
            return False

    def step(self, control):

        # set control
        self._control = control

        # integrate one step
        self._integrator.step()

        # extract state and time
        state, time = self._integrator.y, self._integrator.t

        # check safety
        safe = self.safe(state)

        # check completion
        done = self.done(state)

        # return next state and time
        return state, time, safe, done

    def simulate(self, control, time=None):

        # reset state and time record
        self.reset()

        # conditions
        safe, done = True, False

        # if given control function
        if callable(control):

            while safe and not done:

                # get control
                u = control(self._integrator.t, self._integrator.y)

                # step one time step
                state, time, safe, done = self.step(u)

                # record
                if safe:
                    self.states = np.vstack((self.states, state))
                    self.times  = np.append(self.times, time)

        # if given control sequence
        elif time is not None and len(control) + 1 == len(time):

            for i in range(len(control)):

                # set control
                u = control[i]

                # set times
                t0 = time[i]
                tf = time[i+1]

                while safe and not done and self._integrator.t != tf:

                    # integrate one step in time
                    s, t, safe, done = self.step(u)

                    # interpolate if time is over
                    if t > tf:
                        t = tf
                        s = self._integrator.dense_output()(t)
                        safe = self.safe(s)
                        done = self.done(s)
                        self._integrator.t = t

                    if safe:
                        self.states = np.vstack((self.states, s))
                        self.times  = np.append(self.times, t)




        if safe and done:
            return True
        else:
            return False

    def plot_traj(self, ax=None):

        if ax is None:
            fig, ax = plt.subplots(1)

        # plot trajectory
        ax.plot(self.states[:,0], self.states[:,1], 'k-')

        # plot environment
        self._environment.plot(ax)

        try:
            return fig, ax
        except:
            pass

    def plot_states(self, ax=None):

        if ax is None:
            fig, ax = plt.subplots(self._dynamics.sdim, sharex=True)

        # plot states
        for i in range(self._dynamics.sdim):
            ax[i].plot(self.times, self.states[:,i], 'k-')
            ax[i].set_ylabel(self._dynamics.syms[i])
        ax[-1].set_xlabel(r'$t \: [s]$')

        try:
            return fig, ax
        except:
            pass



if __name__ == '__main__':

    # instantiate mission
    mis = Mission()

    # define random controller
    def controller(time, state):
        u =  np.random.uniform(-0.1, 0.2)
        return u

    # simulate with that controller
    mis.simulate(controller)

    # plot
    fig, ax = mis.plot_traj()
    plt.show()
    fig.savefig('../doc/control_example.svg', bbox_inches=None)

    fig, ax = mis.plot_states()
    plt.show()
    fig.savefig('../doc/control_example.svg', bbox_inches=None)
