# Drake Simulation

This page illustrates how the Drake Simulation works. We need to introduce 3 components that drake use. That's **Simulator**, **SceneGraph** and **Diagram**. Drake organizes these parts with the principles similar to Matlab Simulink, that you could connect these blocks with Drake builder to easily construct your system.

### Diagram

To explain what is `Diagram`, we could look at how Simulink works. Matlab Simulink is a simulation tool that everybody uses to simulate control systems. Simulink is a big graph composed of many connected systems. We could navigate into a system and explore subsystems within the system.

`Diagram` is the big graph in drake. `Diagram` compose of the `MultibodyPlant`, controllers and other useful blocks. Like Simulink, the diagram determines how the system is constructed, what each block is, how they are connected. Drake has `DiagramBuilder` to help glue the system blocks together, it adds system blocks into diagram and connects input and output ports of block together.

_Thinking: what is the information and data format transmitted in between the ports?_

For a robotic system, there is a special system block that represents  each robot in diagram. This system is called `MultibodyPlant`. The `MultibodyPlant` is a complex class that has all the information and data related to robot.

{% page-ref page="drake-multibody.md" %}

### SceneGraph

`SceneGraph` is the visualization and collision checking tool. 

#### Collision Checking

Before simulation, `SceneGraph` is initiated and connected to `MultibodyPlant`. 

Once connected, `SceneGraph` would gives the information of whether two object  collide and what is the distance between two object, given the state of `MultibodyPlant`. It is the `MultibodyPlant` who decide how much force is generated in between objects given the collision information. `MultibodyPlant` could model this collision as a soft contact or a fierce force. 

#### Visualization

To visualize the robot, `MultibodyPlant` should be registered to the `SceneGraph`. The `SceneGraph` would then send rendering message to another process called `drake_visualizer` using LCM.

`drake_visualizer` would help render the robot and other objects.

### Simulator

The `Simulator` takes in the whole system diagram and runs the simulation. Using the robot dynamics and environment forces, the `Simulator` computes the state change. It then runs numerical integration to calculate the next system state, and write the states back to the diagram data bank. It keep updating the states until the simulation finishes.

## Steps: from URDF to a moving robot

1. Import URDF or SDF file to create the robot `MultibodyPlant` in `diagram`.
2. Connect the `MultibodyPlant` input with torque input block, which could be controller block or signal source block.
3. Register the robot into `SceneGraph` for visualization, use builder to connect the `SceneGraph` and `MultibodyPlant` for collision checking.
4. Create `Simulator` to simulate the `diagram`.
5. Compile and run. Open `drake_visualizer` to see the result.

A complete example of this process could be found below.

{% page-ref page="../drake-controllers/try-out-pid-controller.md" %}

