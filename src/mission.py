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

        # arbitrary control for integrator initialisation
        self._control = 0

        # numerical integrator
        self._integrator = ODE(
            self._eom,
            0,
            np.hstack((self.origin, np.zeros(self._dynamics.sdim - 2))),
            500,
            jac = self._jac,
            atol=1e-8,
            vectorized=True,
            max_step=0.1
        )

        # reset mission
        self.reset()

    def set(self, state, time):

        # set state and time for user
        self.state = np.array(state, float)
        self.time  = float(time)

        # set numerical integrator state and time
        self._integrator.y = np.array(self.state, float)
        self._integrator.t = float(self.time)

    def reset(self):

        # initial state and time
        s0 = np.hstack((self.origin, np.zeros(self._dynamics.sdim - 2)))
        t0 = 0

        # set to nominal conditions
        self.set(s0, t0)

        # reset records
        self.states   = np.array([s0], float)
        self.times    = np.array([t0], float)
        self.controls = np.empty((1, 0), float)

    def record(self, state, time, control):

        # record state, time, and control
        self.states   = np.vstack((self.states, state))
        self.times    = np.append(self.times, time)
        self.controls = np.append(self.controls, control)

    def _eom(self, t, state):
        return self._dynamics.eom_state(state, self._control)

    def _jac(self, t, state):
        return self._dynamics.eom_state_jac(state, self._control)

    def safe(self, p0, p1=None):

        # if only given one position
        if p1 is None:
            return self._environment.safe(p0)
        # if given a line
        else:
            return self._environment.safe(np.vstack((p0, p1)))

    def done(self, p):
        delta = np.linalg.norm(p - self.target)
        if delta < 0.1:
            return True
        else:
            return False

    def step(self, control, inplace=False, verbose=False):

        # set the control
        self._control = control

        # store the starting state and time
        s0, t0 = self._integrator.y, self._integrator.t

        # take one integration step
        self._integrator.step()

        # extract new state and time
        s1, t1 = self._integrator.y, self._integrator.t

        # do not set internal state
        if not inplace:
            self.set(s0, t0)
        else:
            pass

        # print the new state if desired
        if verbose:
            print('State', s1, 'Time', t1)

        # check safety of new position
        if not self.safe(s1[:2]):
            safe = False
        # if new position is safe, check for intersection
        elif not self.safe(s0[:2], s1[:2]):
            safe = False
        # if its all good in the hood
        else:
            safe = True

        # check if new position is near target
        done = self.done(s1[:2])

        # return the resulting conditions
        return s1, t1, safe, done

    def simulate(self, control, time=None, obs=True, verbose=False):

        # reset state and time record
        self.reset()

        # set conditions
        safe, done = True, False

        # if given control function
        if callable(control):

            while (safe and not done and self._integrator.status is 'running' and obs) or \
                  (not done and self._integrator.status is 'running' and not obs):

                # compute control
                u = control(self._integrator.t, self._integrator.y)

                # step one time step
                s, t, safe, done = self.step(u, inplace=True)

                # if overshoot
                if t > self._integrator.t_bound:
                    t = self._integrator.t_bound
                    s = self._integrator.dense_output()(t)
                    self.set(s, t)

                if verbose:
                    print('State', s, 'Time', t)

                # record
                self.record(s, t, u)


        # if given control sequence
        elif time is not None and len(control) + 1 == len(time):

            for i in range(len(control)):

                # set control
                u = control[i]
                # seed time
                t = time[i]

                while (safe and not done and self._integrator.status is 'running' and obs and t < time[i+1]) or \
                      (not done and self._integrator.status is 'running' and not obs and t < time[i+1]):

                    # integrate one step in time
                    s, t, safe, done = self.step(u, inplace=True)

                    # if overshoot
                    if t > time[i+1]:
                        t = time[i+1]
                        s = self._integrator.dense_output()(t)
                        self.set(s, t)

                    if verbose:
                        print('State', s, 'Time', t)

                    # record
                    self.record(s, t, u)

                # if unsafe or done within trajectory, break it
                if ((not safe and done) and obs) or (done and not obs):
                    break

                # otherwise keep looping
                else:
                    pass

        # if succesful
        if safe and done:
            return 1

        # if unsuccesful
        else:

            # origin distance from target
            D = np.linalg.norm(self.target - self.origin)

            # car distance to target
            d = np.linalg.norm(self.target - s[:2])

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

    # random control function
    cf = lambda t, s: np.random.uniform(-0.1, 0.2)

    # random control sequence
    u = np.random.uniform(-0.1, 0.2, 20)
    t = np.linspace(0, 100, len(u) + 1)

    # simulate with control function
    print(mis.simulate(cf, verbose=True))
    #print(mis.simulate(u, t, verbose=True, obs=False))

    # visualise
    fig, ax = mis.plot_traj()
    fig, ax = mis.plot_records()
    plt.show()
