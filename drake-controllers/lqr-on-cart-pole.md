# LQR on Cart Pole

Linear Quadratic Regulator \(LQR\) is an optimal control method. Cart-Pole is a canonical model with one prismatic joint connecting the ground and cart and one revolute joint connect the cart and bar.

![Cart-Pole tracking state](../.gitbook/assets/cart_pole_tracking.gif)

### A brief explanation of LQR

LQR controller is a gain matrix K, which maps income system state to control. The gain matrix K comes from the solution of Riccati Equation. 

Optimal control is all about minimizing cost. We formulate the state tracking control problem into a cost minimizing problem, which is equal to solve the Riccati Equation. What's amazing is, the solution of Riccati Equation gives us the optimal controller that generates the minimum cost.

So given the system and cost definition, we could compute the optimal controller K.

To learn more about Cart-Pole model and LQR, find the notes from [Russ Tedrake's Underactauted Robotics](http://underactuated.mit.edu/underactuated.html?chapter=acrobot).

### Get the code

Get the code from my repo, we need to modify the original SDF file to get a fully actuated Cart Pole. And we need a fixed DARE solver in Drake.

### Run the demo

```text
bazel run //examples/multibody/cart_pole:cart_pole_lqr
```

### The code explained

### Implementation notes

LQR Controller in Drake is an affine system. Because LQR is about error dynamics, so the gain matrix K of LQR has to get the state error, and output the control error. With affine system, LQR could get state directly and output control directly, the conversion to state error and convert back to control is handled within the affine system.

Indexing is very important in Drake. A correct indexing is required to get the system work correctly. LQR is using Drake Index on control rather than generalized tree index.

