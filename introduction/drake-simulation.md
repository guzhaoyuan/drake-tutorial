# Drake Simulation

This page illustrates how the Drake Simulation works. We need to introduce 3 components that drake uses. They are **Simulator**, **SceneGraph** and **Diagram**.

### Diagram

To explain what a `diagram` is, we could look at how Matlab Simulink works. Matlab Simulink is a simulation tool. Simulink's main graph is composed of connected systems. A complex system can host multiple subsystems.

`diagram` is the main graph of drake. `diagram` is composed of systems like `MultibodyPlant`, controllers and other useful blocks. Like Simulink, the `diagram` determines how the system is constructed, what each block is, how they are connected. Drake has a`DiagramBuilder` class to help glue the system blocks together, it adds system blocks into diagram and connects input and output ports of block together.

_Thinking: what is the information and data format transmitted in between the ports?_

For a robotic system, there is a special system block that represents  all the robots in `diagram`. This system is called `MultibodyPlant`. The `MultibodyPlant` is a huge class that has all the parameters and data related to robot.

{% page-ref page="drake-multibody.md" %}

### SceneGraph

`SceneGraph` is the visualization and collision checking tool. 

#### Collision Checking

Before simulation, `SceneGraph` is initiated and connected to `MultibodyPlant`. 

During simulation, `SceneGraph` would give the information of whether two objects collide and what is the distance between two object, given the state input from `MultibodyPlant`. Then the `MultibodyPlant` decides whether the collision is a soft contact or a fierce crash, how much force is generated in between objects given the collision information.

#### Visualization

To visualize the robot, `MultibodyPlant` should be registered to the `SceneGraph`. The `SceneGraph` would then send rendering message to another process called `drake_visualizer` using LCM.

`drake_visualizer` would handle the rendering job. It would draw the robot, frames, arrows per request.

### Simulator

The `Simulator` takes the whole system `diagram` and runs the simulation. Using the robot dynamics equation of motion and environment forces, the `Simulator` computes the state change. It then runs numerical integration for continuous system or state update for discrete system, to calculate the next system state, and write the states back to the diagram's corresponding `context`. It keeps updating the states until the simulation finishes.

## Steps: from URDF to a moving robot

1. Import URDF or SDF file to create the robot `MultibodyPlant` in `diagram`.
2. Connect the `MultibodyPlant` input with torque input block, which could be controller block or signal source block.
3. Register the robot into `SceneGraph` for visualization, use `builder` to connect the `SceneGraph` and `MultibodyPlant` for collision checking.
4. Create `Simulator` to simulate the `diagram`.
5. Compile and run. Open `drake_visualizer` to see the result.

A complete example of this process could be found below.

{% page-ref page="../drake-controllers/try-out-pid-controller.md" %}

