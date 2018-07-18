# Christopher Iliffe Sprague
# sprague@kth.se

from mission import Mission
import numpy as np, matplotlib.pyplot as plt
from environment import Environment

def control_function_example():

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
    print(mis.simulate(control_function, obs=True))

    # visualise
    mis.plot_traj()
    mis.plot_records()
    plt.show()

def control_sequence_example():

    # instantiate mission
    mis = Mission()

    # steering pointer function
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

    states = np.empty((0, 3), float)
    safe, done = True, False
    while safe and not done:

        # try control in direction of target
        s, t, safe, done = mis.step(pointer(mis.state))

        # set the state and time
        mis.set(s, t)
        states = np.vstack((states, s))

    fig, ax = mis.plot_traj()
    ax.plot(states[:,0], states[:,1], 'k-')

    plt.show()

if __name__ == '__main__':
    #control_function_example()
    control_sequence_example()
