# To get started

You are excited to simulate your beloved robot with your brand new robotics software. You want to compute kinematics and dynamics. You want to do advanced motion planning because you heard Boston Dynamics uses Quadratic Programming to do back flips. You are in the right place, Drake is the software to get you started.

### Install drake

Although there are binary and [python bindings](https://drake.mit.edu/python_bindings.html#using-python-bindings) of Drake, I recommend you compile Drake from its C++ source code \(this tutorial focuses only on the C++ version of Drake\).

To download and install Drake, follow these [installation](https://drake.mit.edu/installation.html) [instructions](https://drake.mit.edu/installation.html). 

Note: Drake is compiled with the [**bazel**](https://bazel.build/) build tool \(developed by Google\). It has similar functions as Cmake.

### Try the following simulation examples

#### Simulate a ball dropping onto a inclined plane

This [README](https://github.com/RobotLocomotion/drake/tree/master/examples/multibody/inclined_plane_with_body) helps you simulate a ball dropping from a certain height onto a inclined plane, and sliding/rolling down the plane.

#### Visualize a KUKA arm

To control the position of a KUKA arm with a slider GUI, follow these instructions. Note: The complete code is [here](https://github.com/RobotLocomotion/drake/blob/f9e34080cf77ddf49370eaa866212e50f245e6d4/manipulation/util/geometry_inspector.py#L9).

```text
cd drake
bazel build //tools:drake_visualizer //manipulation/util:geometry_inspector
# Terminal 1
./bazel-bin/tools/drake_visualizer
# Terminal 2
./bazel-bin/manipulation/util/geometry_inspector \
    ./manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf
```

#### Explore more examples

Ideally, you now have learned how to visualize and run a simulation. You are ready to enjoy more examples in the [_drake/examples_](https://github.com/RobotLocomotion/drake/tree/master/examples) folder, such as the [allegro hand example](https://github.com/RobotLocomotion/drake/blob/master/examples/allegro_hand/run_allegro_constant_load_demo.cc), in which you import an SDF model and run its simulation.

### Where to go from here

#### Understand Drake concepts

{% page-ref page="introduction/drake-concept.md" %}

#### Try using controller to control the robot

{% page-ref page="thing-to-do-in-drake/try-out-pid-controller.md" %}

