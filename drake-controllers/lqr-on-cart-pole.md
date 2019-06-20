# LQR on Cart Pole

Linear Quadratic Regulator is a optimal control method. LQR in Drake is actually a gain matrix K, computed using Riccati Equation. Based on linearized state space system parameter A, B at desired state and assigned penalty matrix Q R, we could formulate the quadratic cost minimization problem into a Riccati Equation solving problem.

![Cart-Pole tracking state](../.gitbook/assets/cart_pole_tracking.gif)

### To prepare the example

Get the code from my repo, we need to modify the original SDF file to get a fully actuated Cart Pole. And we need a fixed DARE solver in Drake.

### Code



### The Code Explained



