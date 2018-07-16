# Robotic Planning Assignment 2018

In this assignment we consider a constant velocity Dubin's car model, in which the steering angle is sole the control parameter, modelled by the system of ordinary differential equations below.

![](doc/eom.svg)

Suppose that we must drive the car between particular positions, while avoiding obstacles (see the diagramme below).

![](doc/env.svg)

## The Mission

In this assignment we'll work with a `Mission` object, which encapsulates the dynamics and environment of this problem.

### Attributes

A `Mission` object has four attributes:
- `origin : np.ndarray(shape=(1,2))`: the initial position of the car, randomly assigned within the area boudaries.
-  `target : np.ndarray(shape=(1,2))`: the target position that the car seeks to drive to.
- `states : np.ndarray(shape=(n,3))`: the record of `n` simulated states, as generated by `step` or `simulate`.
- `times : np.ndarray(shape=(1,n))`: the record of `n` simulated times, as generated by `step` or `simulate`


### Methods

| Methods         | Argument Types | Returns                                                                                                                                                                                                                   | Description                                                                              |
|-----------------|----------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------|
| `reset()`       |                |                                                                                                                                                                                                                           | Resets state and time record                                                             |
| `step(control)` | `float`        | <li>`state : np.ndarray(shape(1,3))` New state <li>`time : float` New time <li>`safe : bool` `True` if new state is within boundries and in collision free space.  <li>`done : bool` `True` if position matches target's. | <li>Simulates the system one step forward in time <li>Adds new state and time to records |

## The task

Using some robotic planning method, construct either a *control function* or a *control sequence* that drives the car between the origin and target positions without intersecting obstacles.

### Control function
A `control function` must take as its arguments a float `time` and a three-dimensional array `state`. The function outputs a float between -1 and 1 `control`.

```python
def control_function(time, state):
  ...
  return control
```

This method will be validated by simulating the car, calling the control function at every moment in time, until the car has reached the target (within some tolerance) or a maximum amount of time has passed.
A **successful** control function will drive the car to the target without intersecting obstacles and within the maximum time.

Here is an example usage:
```python
>>> time  = 1
>>> state = [0, 10, 0] # x, y, theta
>>> control_function(time, state)
0.8124
```

### Control sequence
This method requires two equally sized arrays, one of the control sequence and the other of the times at which the controls were executed.

Here is an example of a control sequence:
```python
controls = [1, 0.8, 1, 0, 0, -0.3]
times    = [0, 2, 4, 5, 6, 7]
```

This method will be validated by simulating the car with constant controls between the times.
A **successful** control sequence will drive the car to the target without intersecting obstacles.

## API
