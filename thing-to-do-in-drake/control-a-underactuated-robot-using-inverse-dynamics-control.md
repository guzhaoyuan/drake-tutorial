# Control a underactuated robot using Inverse Dynamics Control

### Why Inverse Dynamics Control, why not PID?

Why not use PID controller? That's a good question. If the robot's joints are decouples and using PID controller for each joint is the way to go. All you need is model-free parameters that corresponds to each joint. In my case, I am simulating a humanoid robot with two arms, the joints and links are deeply coupled that model-free method does not work well here.

The [Inverse Dynamics Controller](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1controllers_1_1_inverse_dynamics_controller.html#details) is a system block that takes in the desired acceleration and spit out whatever the general control force is. Drake is handy because it has block that does all the inverse dynamics for you, so all you need is to pass the PID parameters into the controller to wrap the feedforward control signal into a feedback control.

However, there is no such thing as a free lunch. The drake Inverse Dynamics Controller assumes the robot is a fully actuated system. For humanoid robot, this is definitely not true. So we have to use some tricks to fool the controller, make it serve for us.

### One possible method

However, it does not handle the underactuated robot. Controlling a underactuated robot using inverse dynamics control is actually achievable using partial feedback linearization and then with some planning algorithm to modify the reference to a more achievable goal. Then we could use Inverse Dynamics Control to solve the problem of controlling the controllable states. This is one way to work around the problem. But now we will talk another solution here.

### We adopt another method.

Here we assume that we only care about the second half part of the possible method. We focus on how to control the controllable states and ignore the side effect introduced by the controlled states \(The uncontrollable states will definitely be affected because of the coupling of states\).

We could play with the reference. We set the uncontrollable states to zero and leave the rest of the controllable states as is. The reason we could do that is under the condition that we do not need to care about the uncontrollable states. If you do care how the controlled states affect the uncontrollable ones, you should not use this method.

The only problem that we have here is if the Inverse Dynamics function would give us a correct feedforward general force considering the dynamics of uncontrollable states. That actually depends on your system. If the your controllable states and uncontrollable states are coupled heavily, purely set uncontrollable state to zero would surely affect the states. If they are not heavily coupled, then go for it, setting zero is not going to affect the controllable state much.

