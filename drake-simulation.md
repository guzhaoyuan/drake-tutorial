# Drake Simulation

> A continuous robotics simulation software needs three part. That's **Forward Dynamics**, **Numerical Integration** and **Visualization**.

This page illustrates how the Drake Simulation works. We need to introduce 3 components that drake use. That's **simulator**, **scene\_graph** and **diagram**. Drake organize these parts as a concept similar in simulink, that you could connect the blocks to easily construct your system.

### scene\_graph

Handles the visualization and collision.

### diagram

Includes the robot multibodyplant and other useful blocks.

### simulator

The simulator takes in the diagram and runs the simulation.

