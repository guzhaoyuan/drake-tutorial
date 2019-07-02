# Drake Concepts

Drake's core library has 3 big parts:

### I. Dynamical Systems Modeling

Dynamical Systems Modeling requires to represent a dynamical system and simulate based on real world physics.

#### System

Drake's system modeling works like Matlab Simulink. Drake constructs complex systems from blocks called `system`. `system` has input/output ports that could be connected with other system. **A `system` block can be a `diagram` or a `leafsystem`.** `leafsystem` is the minimum build block and a `diagram` is composed of multiple `leafsystem` or `diagram`.

`leafsystem` functions as basic components in robotics systems, like signals, sensors, controllers, planners, etc.

#### Diagram

Drake uses `diagram` to represent systems that internally have several connected systems that function as a whole. `diagram` itself is a system and can be nested. Root diagram contains all the sub-systems and sub-diagrams.

Systems can be connected through their input/output ports. Connected systems could also be formed into a `diagram`.

#### Context

`context` is data of system states and parameters cached in a separate place. Each `diagram` and each `system` has its own `context`. The `context` and the `diagram` are the only two information a `simulator` needs for simulation. Given the `context`, all methods called on a `system` should be completely deterministic and repeatable \(ref. [Underactuated Robotics textbook](http://underactuated.csail.mit.edu/underactuated.html?chapter=systems)\).

Drake has method `diagram->CreateDefaultContext()` which creates the context with default value for all the subsystems. Things such as the initial state and the initial time can directly be set using `context` before simulation starts.

![](../.gitbook/assets/diagram.png)

A context could have continuous state, discrete state and abstract variable. Based on the variable type, the simulator would update the context data by either numerically integrate the continuous derivative or [update the state by state space dynamics](https://drake.mit.edu/doxygen_cxx/group__discrete__systems.html).

#### Simulation

Drake is a simulation software. The Drake `simulator` takes in the system `diagram` together with its `context`, to simulate by updating parameters such as integral continuous state derivatives, compute discrete state updates, allocates the various outputs of a `system`, etc.

### II. Mathematical Programs Solving

Drake incorporates famous and useful optimization tools, for example, Gurobi, SNOPT, IPOPT, SCS, MOSEK. These tools help to solve mathematical problem in robotics, especially areas like motion planning and control.

To use Mathematical Programming, there is a [very good starting point](https://colab.research.google.com/github/RussTedrake/underactuated/blob/master/src/mathematical_program_examples.ipynb) written in python. Same idea applies to C++.

### III. Multibody Kinematics and Dynamics

Multibody means multiple rigid bodies connected in a tree structure. For robotics systems, root `diagram` has a unique `leafsystem` called `MultibodyPlant`. `MultibodyPlant` internally uses rigid body tree algorithms to compute the robot kinematics dynamics jacobian, etc. 

`MultibodyPlant` is a `system`. So it has input/output port that could be connect to other systems like controllers and visualizer.

## Tools that Drake uses

### 1. Eigen

[Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) is a C++ library with linear algebra operations and algorithms.

#### AutoDiff

A convenient technique to compute Derivative. Computing Integral is trivial and computing Derivative is non-trivial. AutoDiff is a good solution that Eigen provides to solve the derivative.

### 2. Lightweight Communications and Marshalling \(LCM\)

LCM is a multi-process communication tool. LCM is everywhere in Drake. It serves as bridge between system ports, so all the communication between systems are transported using LCM, and thus can be inspected by LCM spy tool.

{% page-ref page="../thing-to-do-in-drake/visualize-data-in-lcm.md" %}

### 3. Tinyxml2

A handy tool to parse XML file, enables Drake to parse URDF and SDF, thus create `MultibodyPlant` for simulation.

### 4. The Visualization Toolkit \(VTK\)

Drake uses [VTK](https://vtk.org/) as geometry rendering tool. The Drake visualizer communicate with the simulation through LCM.

