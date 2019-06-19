# Drake Concepts

Drake's core library has 3 big parts:

### Dynamical Systems Modeling

This is Drake simulation. Drake model complex systems by building up from small blocks called `system`. Connected`system` is called `diagram`.

#### Diagram

Drake's system modeling is like Matlab Simulink. Drake uses abstract block `diagram` to represent different systems. There are connections between blocks representing the input/output relationships between those `system`. The diagram could be nested.

#### Context

`context` is cached data of system states, which is required for simulation. Each diagram and each system has its own context. The context has the same structure as the diagram.

![](../.gitbook/assets/diagram.png)

#### Simulation

Drake is a simulation software. The Drake `simulator` takes in the system `diagram` together with its `context`, to perform Forward Dynamics + Numerical integration thus simulate the whole system.

### Mathematical Programs Solving

Drake incorporates famous and useful optimization tools, for example, Gurobi, SNOPT, IPOPT, SCS, MOSEK. These tools helps to solve mathematical problem in robotics, especially in motion planning.

### Multibody Kinematics and Dynamics

For robotics system, `diagram` has a unique `system` called `MultibodyPlant`. `MultibodyPlant` is a rigid body system. This class is rich with methods that compute robot kinematics dynamics jacobian, etc.

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

