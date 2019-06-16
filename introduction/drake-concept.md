# Drake Concepts

Drake is a huge software library with a dedicated team actively supporting robotics developers.

There are several big parts of Drake:

### Modeling Dynamical Systems

This is the simulation part. Drake model complex systems by building up from small blocks.

#### Diagram

Just like Simulink model, at it's core, Drake uses an abstract block diagram representation to model a dynamic system. Blocks generally represent different systems, and the connections between blocks represent the input/output relationships between those systems. ****The basic idea is to make system blocks and connect them together.

#### Simulation

Drake is a simulation software. The simulator could take in the system diagram and perform Forward Dynamics + Numerical integration, to simulate the whole system. It also have a APIs to interact with objects in the simulator. 

### Solving Methematical Programs

Drake incorporated famous and useful optimization tools, for example, Gurobi, SNOPT, IPOPT, SCS, MOSEK. This part helps to solve methematical problem in robotics, especially in motion planning.

### Multibody Kinematics and Dynamics

Multibody represents systems composed of multiple rigid body. This class is rich with methods that handles robot kinematics and dynamics calculation. Multibody acts as a subsystem in Diagram during simulation.

Utility tools: Utility tools includes visualization tools to display the simulation result.

## Tools that Drake use

#### Eigen

[Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) is a C++ library with linear algebra operations and algorithms.

#### LCM

LCM is a communication tool for multi-process communication. LCM is everywhere in Drake. It serves as bridge between system ports, so all the communication between systems can be inspected by LCM spy tool.

{% page-ref page="../thing-to-do-in-drake/visualize-data-in-lcm.md" %}

#### Tinyxml2

A handy tool to parse XML file, enables Drake to parse URDF and SDF, thus create `MultibodyPlant` for simulation.

#### The Visualization Toolkit \(VTK\)

Drake uses [VTK](https://vtk.org/) as geometry rendering tool. The Drake visualizer communicate with the simulation through LCM.

