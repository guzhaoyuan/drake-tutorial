---
description: >-
  An introduction to Trajectory Optimization and a tutorial on Direct
  Transcription.
---

# \[WIP] Trajectory Optimization 1 - Direct Transcription

Trajectory Optimization is a mathematical technique that generates optimal behaviors for robots. It is widely used in industries, such as humanoid robot, autonomous driving. There are different methods to do trajectory optimization. In this tutorial, we will cover Direct Transcription, Direct Collocation and Direct Shooting methods.

[Matthew Kelly](http://www.matthewpeterkelly.com/) that gives a good [tutorial ](https://epubs.siam.org/doi/pdf/10.1137/16M1062569)on Trajectory Optimization. Also Russ Tedrake have a [note in the underactuated robotics course](http://underactuated.mit.edu/underactuated.html?chapter=trajopt) that gives a introduction on these techniques.

The trajectory optimization is an optimization problem that requires mathematical programming. Drake is good at solving mathematical program problems. Drake documentation provides a [tutorial on Drake mathematical programming](https://mybinder.org/v2/gh/RobotLocomotion/drake/master?filepath=tutorials/mathematical\_program.ipynb).

### Direct Transcription

Direct Transcription is an intuitive method that discretize the trajectory and transform the optimization problem into a nonlinear programming problem that has this form:

$$
\min. \quad f(z)\\
{\rm s.t.} \quad g(z) = 0
$$

The $$f(z)$$ is goal function and $$g(z)$$ is the constraint. In the robotics setting, the z usually includes the state and control input, while the constraint involves the dynamics equation of motion and state limits.

For a continuous trajectory, direct method discretize it into points at discrete times, which are called **break points** or **knot points**.

We give the solver a initial guess for all state and control at knot points. By solving this optimization problem, the resultant z will be our optimal states and its corresponding control that obeys the dynamical constraints.

### Moving Box Example

Here we focus on using the Drake toolbox to solve mathematical programming for the optimal trajectory.

We use a simple example from Kelly's tutorial. we want to push and pull a box on a friction-free desk and move it to the desired state with minimum efforts.

### Useful Resources

\[1] Kelly, M. (2017). An introduction to trajectory optimization: How to do your own direct collocation authors&#x20;matthew kelly. SIAM Review, 39.
