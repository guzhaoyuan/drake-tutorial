# Drake Concepts

First of all, Drake is a huge software library with a dedicated team who is actively supporting developers to use Drake.

There are several big part of Drake.

* Multibody, used to create robot model from URDF or SDF or API.
* Math, a mathematics library with a lot of convenient operations needed in Robotics.
* Solver, includes some famous and useful optimization tools, like Gurobi, SNOPT, IPOPT, SCS, MOSEK.
* Simulator, Forward Dynamics + Numerical integrator + visualization + API to interact with things in the simulator.
* Utility tools: like visualization tools that the simulator would use.

## Key Concept

#### LCM

There is a communication tool that drake use for multi-process communication called LCM.

#### Multibody

The Multibody is a important class that represent the robot. It has plenty of functions to help ease the computation and development of robots.

{% page-ref page="drake-multibody.md" %}



