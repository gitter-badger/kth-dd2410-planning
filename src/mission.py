# Christopher Iliffe Sprague
# sprague@kth.se

import numpy as np
from scipy.integrate import RK45 as ODE
from dynamics import Dynamics
from environment import Environment
import matplotlib.pyplot as plt


class Mission(object):

    def __init__(self, dyn=None, env=None):

        if dyn is None:
            dyn = Dynamics(1)
        if env is None:
            env = Environment(50, 30, 1, 20)

        # dynamics
        self._dynamics = dyn

        # environment
        self._environment = env

        # origin and target
        self.origin = self._environment.p0
        self.target = self._environment.pf
        self.control = 0

        # numerical integrator
        self._integrator = ODE(
            self._eom,
            0,
            np.hstack((self.origin, np.zeros(self._dynamics.sdim - 2))),
            500,
            jac = self._jac,
            atol=1e-10,
            rtol=1e-5,
            vectorized=True,
            max_step=0.1
        )

        # reset mission
        self.reset()

    def set(self, state, time):

        # set mission state and time
        self.state = np.array(state, float)
        self.time  = float(time)

        # set numerical integrator state and time
        self._integrator.y = np.array(state, float)
        self._integrator.t = float(time)

    def reset(self):

        # current conditions
        self.state   = np.hstack((self.origin, np.zeros(self._dynamics.sdim - 2)))
        self.time    = 0
        self.control = 0
        self.safe    = True
        self.done    = False

        # reset integrator
        self.set(self.state, self.time)

        # reset records
        self.states   = np.array([self.state], float)
        self.times    = np.array([self.time], float)
        self.controls = np.empty((1, 0), float)

    def _eom(self, t, state):
        return self._dynamics.eom_state(state, self.control)

    def _jac(self, t, state):
        return self._dynamics.eom_state_jac(state, self._control)

    def issafe(self, position):
        return self._environment.safe(position)

    def isdone(self, position):
        delta = np.linalg.norm(position - self.target)
        if delta < 0.1:
            return True
        else:
            return False

    def step(self, control, verbose=False):

        # set control
        self.control = control

        # old state

        # old position
        p0 = self._integrator.y[:2]

        # integrate one step
        self._integrator.step()

        # extract state and time
        state, time = self._integrator.y, self._integrator.t

        if verbose:
            print('State', state, 'Time', time)

        # new position
        p1 = state[:2]

        # check safety of point
        if not self.issafe(p1):
            safe = False
        else:
            # check safety of segment
            safe = self.issafe(np.vstack((p0, p1)))

        # check completion
        done = self.isdone(p1)

        # return next state and time
        return state, time, safe, done

    def simulate(self, control, time=None, obs=True, verbose=False):

        # reset state and time record
        self.reset()

        # if given control function
        if callable(control):

            while (self.safe and not self.done and self._integrator.status is 'running' and obs) or \
                  (self._integrator.status is 'running' and not obs and not self.done):

                # get control
                self.control = control(self._integrator.t, self._integrator.y)

                # step one time step
                self.state, self.time, self.safe, self.done = self.step(self.control)

                # if overshoot
                if self._integrator.t > self._integrator.t_bound:
                    self.time = self._integrator.t_bound
                    self.state = self._integrator.dense_output()(self.time)

                if verbose:
                    print('State', self.state, 'Time', self.time)

                # record
                self.states   = np.vstack((self.states, self.state))
                self.times    = np.append(self.times, self.time)
                self.controls = np.append(self.controls, self.control)

        # if given control sequence
        elif time is not None and len(control) + 1 == len(time):

            for i in range(len(control)):

                # set control
                self.control = control[i]

                # set boundary time
                self._integrator.t_bound = time[i+1]

                while (self.safe and not self.done and self._integrator.status is 'running' and obs) or \
                      (self._integrator.status is 'running' and not obs and not self.done):

                    # integrate one step in time
                    self.state, self.time, self.safe, self.done = self.step(self.control)

                    # if overshoot
                    if self._integrator.t > self._integrator.t_bound:
                        self.time = self._integrator.t_bound
                        self.state = self._integrator.dense_output()(self.time)
                        self._integrator.t = self.time
                        self._integrator.y = self.state

                    if verbose:
                        print('State', self.state, 'Time', self.time)

                    self.states   = np.vstack((self.states, self.state))
                    self.times    = np.append(self.times, self.time)
                    self.controls = np.append(self.controls, self.control)

                if ((not self.safe or self.done) and obs) or (self.done and not obs):
                    break
                elif self._integrator.status is 'finished':
                    self._integrator.status = 'running'


        if self.safe and self.done:
            return 1
        else:

            # origin distance from target
            D = np.linalg.norm(self.target - self.origin)

            # car distance to target
            d = np.linalg.norm(self.target - self.state[:2])

            # return percent distance acheived
            return 1 - d/D


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

    def plot_records(self, ax=None):

        if ax is None:
            fig, ax = plt.subplots(self._dynamics.sdim + 1, sharex=True)

        # plot states
        for i in range(self._dynamics.sdim):
            ax[i].plot(self.times, self.states[:,i], 'k-')
            ax[i].set_ylabel(self._dynamics.syms[i])

        # plot controls
        ax[-1].plot(self.times[:-1], self.controls, 'k-')
        ax[-1].set_ylabel(r'$\phi \: [rad]$')
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

    # define random control sequence
    u = np.random.uniform(-0.1, 0.1, 100)
    t = np.linspace(0, 100, len(u) + 1)

    # simulate with that controller
    print(mis.simulate(u, t, obs=True))
    #print(mis.simulate(controller, obs=True, verbose=True))

    # plot
    mis.plot_traj()
    mis.plot_records()
    plt.show()
