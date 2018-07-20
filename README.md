# Robotic Planning Assignment 2018

In this assignment we consider a constant velocity Dubin's car model, in which the steering angle is the sole control parameter, modelled by the following system of ordinary differential equations.

![](doc/eom.svg)

Suppose that we must drive the car between particular positions, while avoiding obstacles (see the diagramme below).

![](doc/env.svg)

## The code

In this assignment we'll work with a `Mission` object, which encapsulates the dynamics and environment of this problem.

### Attributes
Attribute           | Type                     | Description |
| ----------------- | ------------------------ | ---------------- |
|`Mission.origin`   | `np.ndarray((1,2))`      | Initial position |
|`Mission.target`   | `np.ndarray((1,2))`      | Target position |
|`Mission.state`    | `np.ndarray(shape(1,3))` | Current state, where <li> `state[0]`: horizontal position <li> `state[1]`: vertical position <li> `state[2]`: heading angle|
|`Mission.time`     | `float`                  | Current time |
|`Mission.states`   | `np.ndarray((n,3))`      | Record of `n` states, where: <li> `states[:,0]`: horizontal positions <li> `states[:,1]`: vertical positions <li>`states[:,2]`: heading angles |
|`Mission.times`    | `np.ndarray((1,n))`      | Record of `n` times |
|`Mission.controls` | `np.ndarray((1,n))`      | Record of `n` controls |

### Methods

| Method | Arguments | Returns | Alterations |
| ------ | --------- | ------- | ----------- |
| `Mission.set(s, t)` | <li> `s : np.ndarray((1,3))`: state <li> `t : float`: time | None | Sets <li> `Mission.state` to `s` <li> `Mission.time` to `t` |
| `Mission.reset()`   | None  | None | Resets <li>`Mission.state` <li> `Mission.time` <li> `Mission.states` <li> `Mission.times` <li> `Mission.controls` |
| `Mission.record(s, t, u)`   |  <li> `s : np.ndarray((1,3))`: state <li> `t : float`: time <li> `u : float`: control  | None |  Appends <li> `s` to `Mission.states` <li> `t` to `Mission.times` <li> `u` to `Mission.controls` |
| `Mission.safe(p0, p1=None)`   |   <li> `p0 : np.ndarray((1,2))`: first position <li> `p1 : np.ndarray((1,2))`: second position |  <li> If only given `p0`: `True` if `p0` is within boundaries and in obstacle free space, `False` otherwise <li> If given both `p0` and `p1`: `True` if line does not intersect neither boundaries nor obstacles, `False` otherwise.|  None |
| `Mission.done(p)`   |  <li> `p : np.ndarray((1,2))`: position  | `True` if `p` is approximately at `Mission.target` |  None |
| `Mission.step(u, inplace=False)`   | <li> `u : float`: control <li> `inplace : bool`: call `Mission.set(s1, t1, u)`  | Tuple containing <li> `s1 : np.ndarray((1,3))`: new state <li> `t1 : float`: new time <li> `safe : bool`: `True` if transition was safe, `False` otherwise <li> `done : bool`: `True` if new position is approximately at target, `False` otherwise | <li> If `inplace` is `True` than `Mission.set(s1, t1, u)`  |

### Implementation
With the `Mission` API, described by the above attributes and methods, it is simple to flexibly implement different. To demonstrate this, consider the following examples.

#### Random control

```python
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
```

### Control pointed at target

```python
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

    # get new conditions and set them
    s1, t1, safe, done = mis.step(u, inplace=True)

    # record conditions
    mis.record(s1, t1, u)

# visualise trajectory
mis.plot_traj()
mis.plot_records()
plt.show()
```

## The task

Using some robotic planning method, utilise the above attributes and methods of the `Mission` object to write a Python script that effectively drives the car between the origin and target.

Create a script like function that returns the records `Mission.states`, `Mission.times`, and `Mission.controls`. Your function should look something like this:

```python
import Mission, numpy as np
def main():
  # instantiate mission
  mis = Mission

  # do stuff
  ...
  ...
  ## maybe some RRTs
  ...
  ...

  # return the records
  return mis.states, mis.times, mis.controls
```

### Validation
A **succesful** function will return a squence of states, time, and controls which together are dynamically feasible, collision free, and arrive to the target.
.
