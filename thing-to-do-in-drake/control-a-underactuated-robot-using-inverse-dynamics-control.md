# Control a underactuated robot using Inverse Dynamics Controller

### Why Inverse Dynamics Control, why not PID?

Why not use PID controller? That's a good question. If the robot's joints are decouples and using PID controller for each joint is the way to go. All you need is model-free parameters that corresponds to each joint. In my case, I am simulating a humanoid robot with two arms, the joints and links are deeply coupled that model-free method does not work well here.

The [Inverse Dynamics Controller](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1controllers_1_1_inverse_dynamics_controller.html#details) is a system block that takes in the desired acceleration and spit out whatever the general control force is. Drake is handy because it has block that does all the inverse dynamics for you, so all you need is to pass the PID parameters into the controller to wrap the feedforward control signal into a feedback control.

However, drake Inverse Dynamics Controller assumes the robot is a fully actuated system, it only accept a fully actuated plant as the controlled plant. For any mobile robot, this is definitely not true. So we have to use some tricks to fool the controller, make it work for us.

### One possible method: Partial Feedback Linearization

Controlling a underactuated robot using inverse dynamics control is actually achievable using partial feedback linearization. The idea is to separate the system states into 2 parts, actuated and unactuated. Then we use planning algorithms to change the desired reference into a more achievable goal for both of these states. We have to compromise here because with some unactuated states, the system is underactuated, meaning we could not track whatever reference we want, but with some planning and optimization, we could modify the reference to make the goal achievable. 

We could then use Inverse Dynamics Controller to solve the problem of tracking the actuated states. This is one way to work around the problem. But now we will talk another solution here.

### We adopt another method.

Here we focus on how to control the actuated states and ignore the unactuated states \(The unactuated states will definitely be affected because of the coupling between states\).

We could weld the robot floating base to ground as a fake `MultibodyPlant`. The robot lost the floating base would become a fully actuated robot and can be used to control by `InverseDynamicsController` in drake. So we would simulate a real MultibodyPlant in simulation, but within each simulate loop. We get rid of the floating base states\(unactuated\) and keep the actuated states\(general position and velocity\), then feed this fully actuated states into the controller to ask for control signal.

The only problem that we have here is if the Inverse Dynamics function would give us a correct feedforward general force based on the dynamics of the fake fully actuated`MultibodyPlant`. If unactuated and actuated states are coupled heavily, there is no guarantee that you unactuated states would stable.

The reason we could do that is under the condition that we do not need to care about the unactuated states\(we know these states would stable or we have other constraint to limit the instability\). If you do care about how the actuated states would affect the unactuated ones, you should not use this method, use partial feedback linearization instead.

