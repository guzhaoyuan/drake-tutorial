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

## How the Simulation works

### scene\_graph 

For visualization and collision detection. It would gives the information of how the two object would collide given the state of the system blocks. It is the multibody's job to model this collision as a soft contact or a fierce force. 

### simulator

For simulate the whole world given the diagram.

### diagram

How the subsystem blocks organize together. It compose of subsystem blocks which is glued together by builder. Builder is an object that could connect different components, basically components that has input or output output. Subsystem blocks has input and output ports and Builder would connect these ports.

_Thinking: what is the information and data format transmitted in between the ports?_

Under diagram, for a robot system, there is a special subsystem that represent the robot itself. This subsystem is called multibodyplant. It is the entry of initialize and setup the robot. 

### steps to import urdf robot into simulation:

From the multibodyplant, we could import urdf or sdf file to create the robot easily, 

and then insert the subsystem into diagram,

Connect the multibodyplant input with some other torque input block, Register the robot into scene\_graph, use builder to connect the scene\_graph and plant

Using Simulator to simulate the diagram.

Open visualizer to see the result.





