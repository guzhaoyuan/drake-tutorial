# Visualize data in LCM

System blocks communicate with opening ports. Data flow from one port to another through LCM pipelines. We could visualize the data.

{% hint style="info" %}
The order of executing the command matters in this example. Pay attention to following the order here.
{% endhint %}

### Open the visualizer

Ensure that you have installed the drake visualizer with

```text
cd drake
bazel build //tools:drake_visualizer
bazel-bin/tools/drake_visualizer
```

### Open LCM data inspector

In another terminal, we open the LCM data inspector by

```text
cd drake
bazel-bin/lcmtypes/drake-lcm-spy
```

### Execute the robot

To visualize data, we need data. Controlling a robot to move is a good way to get data flowing. We start a KUKA arm simulation in another terminal:

```text
bazel-bin/examples/kuka_iiwa_arm/kuka_simulation
```

![KUKA arm in Drake simulation](../.gitbook/assets/screenshot-from-2019-06-13-10-24-19.png)

Then in another terminal we open up a "planner and runner".

```text
bazel-bin/examples/kuka_iiwa_arm/kuka_plan_runner
```

Finally we need to send command to the planner, it will run the KUKA arm.

```text
bazel-bin/examples/kuka_iiwa_arm/move_iiwa_ee -x 0.8 -y 0.3 -z 0.25 -yaw 1.57
```

You should see the robot arm moving with the commanding data in the LCM inspector change simultaneously. For more detailed data stream, you can double click on each LCM channel.

![The data channels on the left show each joint&apos;s control command](../.gitbook/assets/untitled.gif)

### Links

The KUKA Manipulation example is [here](https://github.com/RobotLocomotion/drake/tree/master/examples/kuka_iiwa_arm).

