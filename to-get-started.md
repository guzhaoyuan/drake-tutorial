# To get started

You get excited about a brand new Robotics Software. You would like the software to simulate your beloved robot in simulation, you want to compute kinematics and dynamics of the robot, you want to do advanced motion planning using Quadratic Programming because you heard that Boston Dynamics is using that to do back flip. You are in the right place, drake is the right software to get you started.

### Install drake

The first thing to do is to install the drake. 

If you want to develop using drake, I would recommend you compile your own software from source, the c++ version is the fully supported. But there are also ways to using the binary versions and using [python bindings](https://drake.mit.edu/python_bindings.html#using-python-bindings). In this tutorial I would only focus on C++ version of drake.

Drake use [**bazel**](https://bazel.build/) as the building tool. It is a building software from google. It has similar functions as Cmake. Follow the [drake documentation for installation](https://drake.mit.edu/installation.html), it will cover the all you would need for installing from source.

### Play with examples

After installation, try run a simulation example.

#### Simulate a ball dropping into a plane

I personally find this [README](https://github.com/RobotLocomotion/drake/tree/master/examples/multibody/inclined_plane_with_body) is helpful to get started, just follow the instructions in the readme, and you would get a inclined plane and a simulation of ball dropping from a certain height.

#### Visualize KUKA arm

After the ball dropping, you could also play with the KUKA arm using a little slider gui. Follow the instructions from the [gui comments](https://github.com/RobotLocomotion/drake/blob/f9e34080cf77ddf49370eaa866212e50f245e6d4/manipulation/util/geometry_inspector.py#L9) to control the position of the robot.

#### Explore more examples

After the above two, you should have learnt how to open visualizer and how to run a simulation. You could have more fun with examples in the [_drake/examples_](https://github.com/RobotLocomotion/drake/tree/master/examples) folder.

The [allegro hand example](https://github.com/RobotLocomotion/drake/blob/master/examples/allegro_hand/run_allegro_constant_load_demo.cc) is a good example. You could learn how to import sdf model and run simulation using that model.

### Where to go from here

{% page-ref page="introduction/drake-concept.md" %}

