# Christopher Iliffe Sprague
# sprague@kth.se

from mission import Mission
import numpy as np, matplotlib.pyplot as plt
from environment import Environment

def control_function_example(obs=True, verbose=True):

    # instantiate mission
    mis = Mission()

    # control function that points at target
    def control_function(time, state):

        # car position
        p = state[:2]

        # relative position to target
        pt = mis.target - p

        # angle to target
        psi = np.arctan2(pt[1], pt[0])

        # car heading angle
        theta = state[2]

        # direct steering angle at target
        phi = psi - theta

        # return the control with sound
        return phi

    # simualte the mission with controller
    print(mis.simulate(control_function, obs=obs, verbose=verbose))

    # visualise
    mis.plot_traj()
    mis.plot_records()
    plt.show()

def control_sequence_example(with_obs=True, verbose=True):

    # instantiate mission
    mis = Mission()

    # returns steering angle in direction of target
    def pointer(state):

        # car position
        p = state[:2]

        # relative position to target
        pt = mis.target - p

        # angle to target
        psi = np.arctan2(pt[1], pt[0])

        # car heading angle
        theta = state[2]

        # direct steering angle at target
        phi = psi - theta

        # return the control with sound
        return phi

    # let's try pointing towards the target normally
    # and point randomly if needed for n steps
    nsteps = 100

    safe, done = True, False
    while safe and not done and mis.time < 200:

        # compute a control pointing at target
        u = pointer(mis.state)

        # try a step, pointing at the target
        s, t, safe, done = mis.step(u)

        # if it was safe, accept it
        if safe:
            print('State', s, 'Time', t)
            mis.set(s, t)
            mis.record(s, t, u)

        # otherwise, rollback n steps and point in random direction
        else:

            # set back to 10 steps ago
            mis.states = mis.states[:-1-nsteps, :]
            mis.times  = mis.times[:-1-nsteps]
            mis.set(mis.states[-1, :], mis.times[-1])

            while True:

                # try random direction
                u = np.random.uniform(-np.pi/2, np.pi/2)
                print('Trying', u)

                # simulate n steps
                i = 0
                while i < nsteps:

                    s, t, safe, done = mis.step(u)
                    if safe:
                        i += 1
                    else:
                        break
                    if done:
                        break

                if safe:
                    break
                else:
                    continue




    # visualise
    mis.plot_traj()
    mis.plot_records()
    plt.show()

def ex1():

    # instantiate mission
    mis = Mission()

    # step until collision or at target
    safe, done = True, False
    while safe and not done:

        # random steering angle
        u = np.random.uniform(-0.1, 0.1)

        # get new conditions
        s1, t1, safe, done = mis.step(u)

        # set the new conditions
        mis.set(s1, t1)

        # record conditions
        mis.record(s1, t1, u)

    # visualise trajectory
    mis.plot_traj()
    mis.plot_records()
    plt.show()

def ex2():

    # instanite mission
    mis = Mission()

    # step until collision or at target
    safe, done = True, False
    while safe and not done:

        # car position
        p = mis.state[:2]

        # relative position to target
        pt = mis.target - p

        # angle to target
        psi = np.arctan2(pt[1], pt[0])

        # car heading angle
        theta = mis.state[2]

        # steering angle towards target
        u = psi - theta

        # get new conditions
        s1, t1, safe, done = mis.step(u)

        # set new conditions
        mis.set(s1, t1)

        # record conditions
        mis.record(s1, t1, u)

    # visualise trajectory
    mis.plot_traj()
    mis.plot_records()
    plt.show()



if __name__ == '__main__':
    #control_function_example()
    #control_sequence_example()
    #ex1()
    ex2()
