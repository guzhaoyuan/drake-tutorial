# Drake Concepts

Drake is a huge software library with a dedicated team actively supporting robotics developers.

There are several big parts of Drake:

* Diagram: Just like Simulink model, at it's core, Drake uses an abstract block diagram representation to model a dynamic system. Blocks generally represent different systems, and the connections between blocks represent the input/output relationships between those systems. ****The basic idea is to make system blocks and connect them together.
  * Multibody, a special system block used to create and represent robot model from URDF/ SDF/API.
* Math: a mathematics library with a lot of convenient operations needed in Robotics.
* Solver: Drake have incorporated famous and useful optimization tools, for example, Gurobi, SNOPT, IPOPT, SCS, MOSEK.
* Simulation: Drake is a simulation software. The simulator could take in the system diagram and perform Forward Dynamics + Numerical integration, to simulate the whole system. It also have a APIs to interact with objects in the simulator.
* Utility tools: Utility tools includes visualization tools to display the simulation result.

## Key Concepts

#### LCM

There is a communication tool that drake use for multi-process communication called LCM.

#### Multibody

The Multibody is a important class that represent the robot. It has plenty of functions to help ease the computation and development of robots.

{% page-ref page="drake-multibody.md" %}



