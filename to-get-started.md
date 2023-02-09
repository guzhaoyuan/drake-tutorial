# To get started

You are excited to simulate your beloved robot with this brand-new Drake software. You want to solve the robot's kinematics and dynamics. You want to calculate advanced motion planning because you heard Boston Dynamics uses Model Predictive Control to achieve backflips. You are in the right place; Drake is the software to get you started.

### Install drake

Although there are binary and [python bindings](https://drake.mit.edu/python\_bindings.html#using-python-bindings) of Drake, I recommend you compile Drake from its C++ source code (this tutorial focuses only on the C++ version of Drake).

To download and install Drake, follow the [source installation instructions](https://drake.mit.edu/from\_source.html).

{% hint style="info" %}
Note: Drake is compiled with the [**bazel**](https://bazel.build/) build tool (developed by Google). It has similar functions as CMake.
{% endhint %}

### After the installation, try the following simulation examples

#### Simulate a ball dropping onto an inclined plane

This [README](https://github.com/RobotLocomotion/drake/tree/master/examples/multibody/inclined\_plane\_with\_body) helps you simulate a ball dropping from a certain height onto an inclined plane, then sliding/rolling down the plane.

#### Visualize a KUKA arm

To control the position of a KUKA arm with a slider GUI, follow these instructions. (Note: The complete code is [here](https://github.com/RobotLocomotion/drake/blob/f9e34080cf77ddf49370eaa866212e50f245e6d4/manipulation/util/geometry\_inspector.py#L9).)

```
cd drake
bazel build //tools:drake_visualizer //manipulation/util:geometry_inspector
# Terminal 1
./bazel-bin/tools/drake_visualizer
# Terminal 2
./bazel-bin/manipulation/util/geometry_inspector \
    ./manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf
```

#### Explore more examples

Ideally, you now have learned how to visualize and run a simulation. You are ready to enjoy more examples under the [_drake/examples_](https://github.com/RobotLocomotion/drake/tree/master/examples) folder, such as the [allegro hand example](https://github.com/RobotLocomotion/drake/blob/master/examples/allegro\_hand/run\_allegro\_constant\_load\_demo.cc), in which you import an SDF model and run its simulation.

### Where to go from here

#### Understand Drake concepts

{% content-ref url="introduction/drake-concept.md" %}
[drake-concept.md](introduction/drake-concept.md)
{% endcontent-ref %}

#### Try using controllers to control a robot

{% content-ref url="drake-controllers/try-out-pid-controller.md" %}
[try-out-pid-controller.md](drake-controllers/try-out-pid-controller.md)
{% endcontent-ref %}

### Useful links

* [Drake Doxygen](http://drake.mit.edu/doxygen\_cxx/index.html#://) (Web-based documentation for C++ source code)
* [Underactuated Robotics Online Textbook](http://underactuated.csail.mit.edu/underactuated.html) (Class taught by Prof. Russ Tedrake that heavily uses Drake)
