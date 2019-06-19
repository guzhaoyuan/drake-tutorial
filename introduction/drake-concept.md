# Drake Concepts

Drake's core library has 3 big parts:

### Dynamical Systems Modeling

Dynamical System modeling is actually to simulate the real world physics.

#### System

Drake model building up complex systems from blocks called `system`. system has input/output ports that could be connected with other system. **A `system` block can be a `diagram` or a `leafsystem`.** A single system is a `leafsystem`, and multiple connected systems are called `diagram` \(and yet, a `diagram` is in fact a system\).

#### Diagram

Drake's system modeling is like Matlab Simulink. Drake uses a `diagram` to represent the whole system. `diagram` is be nested with child `leafsystem` and `diagram`. There are connections between blocks representing the input/output relationships between those `system`. 

#### Context

`context` is cached data of system states and parameters, which is required for simulation. Each diagram and each system has its own `context`. The `context` and the `diagram` are everything `simulator` need for simulation, and given the `context` all methods called on a `system` should be completely deterministic and repeatable \(ref. [Underactuated Robotics textbook](http://underactuated.csail.mit.edu/underactuated.html?chapter=systems)\).

There is a [method](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_system.html#ab4e6ee413f4f47a20f6dcc2cbd831b88) to create a default value for all `context`. Things such as the initial state and the initial time can directly be set using `context` for that `system` before running the simulation.

![](../.gitbook/assets/diagram.png)

#### Simulation

Drake is a simulation software. The Drake `simulator` takes in the system `diagram` together with its `context`, to update parameters such as the continuous time derivatives, discrete state updates, allocates the various outputs of a `system`, etc.

### Mathematical Programs Solving

Drake incorporates famous and useful optimization tools, for example, Gurobi, SNOPT, IPOPT, SCS, MOSEK. These tools helps to solve mathematical problem in robotics, especially in motion planning.

### Multibody Kinematics and Dynamics

Multibody is a term meaning rigid body connected in a tree structure. For robotics systems, `diagram` has a unique `system` called `MultibodyPlant`. `MultibodyPlant` internally use a rigid body tree algorithms to compute the robot kinematics dynamics jacobian, etc. And because `MultibodyPlant` is a system, it has input/output port that could be connect to controller and visualizer to actually control and visualize the system.

## Tools that Drake use

#### Eigen

[Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) is a C++ library with linear algebra operations and algorithms.

#### Lightweight Communications and Marshalling \(LCM\)

LCM is a multi-process communication tool. LCM is everywhere in Drake. It serves as bridge between system ports, so all the communication between systems are transported using LCM, and thus can be inspected by LCM spy tool.

{% page-ref page="../thing-to-do-in-drake/visualize-data-in-lcm.md" %}

#### Tinyxml2

A handy tool to parse XML file, enables Drake to parse URDF and SDF, thus create `MultibodyPlant` for simulation.

#### The Visualization Toolkit \(VTK\)

Drake uses [VTK](https://vtk.org/) as geometry rendering tool. The Drake visualizer communicate with the simulation through LCM.

