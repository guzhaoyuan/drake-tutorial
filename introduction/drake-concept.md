# Drake Concepts

First of all, drake is a huge software library with a dedicated team who is actively supporting developers to use drake.

There are several big part of drake.

* Multibody, used to create robot model from urdf or sdf or api.
* Math, a mathematics library with a lot of convenient operations needed in Robotics
* Solver, includes some famous and useful optimization tools, like Gurobi, SNOPT, IPOPT, SCS, MOSEK.
* Simulator, Forward Dynamics + Numerical integrator + visualization + API to interact with things in the simulator
* Handy tools: like visualization tools that the simulator would use.

## Key Concept

#### LCM

There is a communication tool that drake use for multi-process communication called LCM.

#### Multibody

The multibody is a import class that represent the robot. It has a lot of functions to help ease the computation and development of robots.

{% page-ref page="drake-multibody.md" %}



