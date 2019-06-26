# Visualize trajectory

Drake does not support visualizing the trajectory yet. To show any customized data format, we need a plugin.

![Visualize trajectory](../.gitbook/assets/screenshot-from-2019-06-25-15-13-33.png)

### Create plugin

You could place your plugin anywhere as long as they load correctly. I would put the plugin file `show_trajectory.py` under _drake/tools/workspace/drake\_visualizer/plugin/_

{% code-tabs %}
{% code-tabs-item title="show\_trajectory.py" %}
```python
# Note that this script runs in the main context of drake-visulizer,
# where many modules and variables already exist in the global scope.
from sets import Set
from director import lcmUtils
from director import applogic
from director import objectmodel as om
from director import transformUtils
from director import visualization as vis
from director.debugVis import DebugData
from six import iteritems
import numpy as np
import robotlocomotion as lcmrobotlocomotion

from drake.tools.workspace.drake_visualizer.plugin import scoped_singleton_func


class FrameChannel(object):

    def __init__(self, parent_folder, channel):
        self._parent_folder = parent_folder
        self._channel = channel
        # Link names that were previously published.
        self._link_name_published = []

    def handle_message(self, msg):
        print("frame channel was called")
        if set(self._link_name_published) != set(msg.link_name):
            # Removes the folder completely.
            self.remove_folder()
            self._link_name_published = msg.link_name

        folder = self._get_folder()

        for i in range(0, msg.num_links):
            name = msg.link_name[i]
            transform = transformUtils.transformFromPose(
                msg.position[i], msg.quaternion[i])
            # `vis.updateFrame` will either create or update the frame
            # according to its name within its parent folder.
            vis.updateFrame(transform, name, parent=folder, scale=0.1)

        # Create map of body names to a list of contact forces
        collision_pair_to_forces = {}
        if msg.num_links > 1:
            for i in range(1, msg.num_links):
                name = msg.link_name[i]
                # msg.position[i] is tuple and can be transformed into np array.
                point1 = np.array(msg.position[i-1])
                point2 = np.array(msg.position[i])
                collision_pair_to_forces[name] = [(point1, point2)]

            for key, list_of_forces in iteritems(collision_pair_to_forces):
                d = DebugData()
                for force_pair in list_of_forces:
                    d.addArrow(start=force_pair[0],
                               end=force_pair[1],
                               tubeRadius=0.005,
                               headRadius=0.01)

                vis.showPolyData(
                    d.getPolyData(), str(key), parent=folder, color=[0, 1, 0])

    def _get_folder(self):
        return om.getOrCreateContainer(
            self._channel, parentObj=self._parent_folder)

    def remove_folder(self):
        om.removeFromObjectModel(self._get_folder())


class FramesVisualizer(object):

    def __init__(self):
        self._name = "Frame Visualizer"
        self._subscriber = None
        self._frame_channels = {}
        self.set_enabled(True)

    def _add_subscriber(self):
        if (self._subscriber is not None):
            return

        self._subscriber = lcmUtils.addSubscriber(
            'DRAKE_DRAW_TRAJECTORY.*',
            messageClass=lcmrobotlocomotion.viewer_draw_t,
            callback=self._handle_message,
            callbackNeedsChannel=True)
        self._subscriber.setNotifyAllMessagesEnabled(True)

    def _get_folder(self):
        return om.getOrCreateContainer(self._name)

    def _remove_subscriber(self):
        if (self._subscriber is None):
            return
        lcmUtils.removeSubscriber(self._subscriber)
        for frame_channel in self._frame_channels:
            frame_channel.remove_folder()
        self._frame_channels.clear()
        self._subscriber = None
        om.removeFromObjectModel(self._get_folder())

    def is_enabled(self):
        return self._subscriber is not None

    def set_enabled(self, enable):
        if enable:
            self._add_subscriber()
        else:
            self._remove_subscriber()

    def _handle_message(self, msg, channel):
        print("trajectory service was called")
        frame_channel = self._frame_channels.get(channel)
        if not frame_channel:
            frame_channel = FrameChannel(
                parent_folder=self._get_folder(), channel=channel)
            self._frame_channels[channel] = frame_channel
        frame_channel.handle_message(msg)


@scoped_singleton_func
def init_visualizer():
    frame_viz = FramesVisualizer()

    # Adds to the "Tools" menu.
    applogic.MenuActionToggleHelper(
        'Tools', frame_viz._name,
        frame_viz.is_enabled, frame_viz.set_enabled)
    return frame_viz


# Activate the plugin if this script is run directly; store the results to keep
# the plugin objects in scope.
if __name__ == "__main__":
    frame_viz = init_visualizer()
```
{% endcode-tabs-item %}
{% endcode-tabs %}

The plugin could be loaded by the `drake_visualizer`, it will call the `_handle_message` every time it gets a message data. The message data is defined as `map{name, transformation}`. The plugin will display all the points as frames and connect the points with arrows.

### Create and send trajectory to visualizer

Add publisher code and `BUILD.bazel` under _drake/examples/viz_. 

This code would create a piecewise cubic polynomial trajectory given several key points and sample points on the trajectory. The points are then packed into message and sent over to the `drake_visualizer`.

{% code-tabs %}
{% code-tabs-item title="trajectory\_publisher.cc" %}
```cpp
/// @file
///
/// This file creates a simple trajectory and visualize it.

/* Examples

PublishFramesToLcm("DRAKE_DRAW_TRAJECTORY", {
    {"X_WF", Eigen::Isometry3d::Identity()},
    {"X_WG", Eigen::Isometry3d::Identity()},
   }, &lcm);
*/

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace eve {

void PublishFramesToLcm(const std::string& channel_name,
                        const std::vector<Eigen::Isometry3d>& poses,
                        const std::vector<std::string>& names,
                        drake::lcm::DrakeLcmInterface* dlcm) {
  DRAKE_DEMAND(poses.size() == names.size());
  drake::lcmt_viewer_draw frame_msg{};
  frame_msg.timestamp = 0;
  int32_t vsize = poses.size();
  frame_msg.num_links = vsize;
  frame_msg.link_name.resize(vsize);
  frame_msg.robot_num.resize(vsize, 0);

  for (size_t i = 0; i < poses.size(); i++) {
    Eigen::Isometry3f pose = poses[i].cast<float>();
    // Create a frame publisher
    Eigen::Vector3f goal_pos = pose.translation();
    Eigen::Quaternion<float> goal_quat =
        Eigen::Quaternion<float>(pose.linear());
    frame_msg.link_name[i] = names[i];
    frame_msg.position.push_back({goal_pos(0), goal_pos(1), goal_pos(2)});
    frame_msg.quaternion.push_back(
        {goal_quat.w(), goal_quat.x(), goal_quat.y(), goal_quat.z()});
  }

  const int num_bytes = frame_msg.getEncodedSize();
  const size_t size_bytes = static_cast<size_t>(num_bytes);
  std::vector<uint8_t> bytes(size_bytes);
  frame_msg.encode(bytes.data(), 0, num_bytes);
  dlcm->Publish("DRAKE_DRAW_TRAJECTORY_" + channel_name, bytes.data(),
                num_bytes, {});
}

void PublishFramesToLcm(
    const std::string& channel_name,
    const std::unordered_map<std::string, Eigen::Isometry3d>& name_to_frame_map,
    drake::lcm::DrakeLcmInterface* lcm) {
  std::vector<Eigen::Isometry3d> poses;
  std::vector<std::string> names;
  for (const auto& pair : name_to_frame_map) {
    poses.push_back(pair.second);
    names.push_back(pair.first);
  }
  PublishFramesToLcm(channel_name, poses, names, lcm);
}

void DoMain() {
  // Design the trajectory to follow.
  const std::vector<double> kTimes{0.0, 2.0, 5.0, 10.0};
  std::vector<Eigen::MatrixXd> knots(kTimes.size());
  Eigen::VectorXd tmp1(3);
  tmp1 << 0, 0, 0;
  knots[0] = tmp1;
  Eigen::VectorXd tmp2(3);
  tmp2 << 1, 1, 0;
  knots[1] = tmp2;
  Eigen::VectorXd tmp3(3);
  tmp3 << 2, -1, 0;
  knots[2] = tmp3;
  Eigen::VectorXd tmp4(3);
  tmp4 << 3, 0, 0;
  knots[3] = tmp4;
  Eigen::VectorXd knot_dot_start = Eigen::VectorXd::Zero(3);
  Eigen::MatrixXd knot_dot_end = Eigen::VectorXd::Zero(3);
  trajectories::PiecewisePolynomial<double> trajectory =
      trajectories::PiecewisePolynomial<double>::Cubic(
          kTimes, knots, knot_dot_start, knot_dot_end);

  std::vector<std::string> names;
  std::vector<Eigen::Isometry3d> poses;
  for (double t = 0.0; t < 10.0; t += 0.1) {
    names.push_back("X" + std::to_string(int(t * 100)));
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = trajectory.value(t);
    poses.push_back(pose);
  }
  lcm::DrakeLcm lcm;

  //  std::vector<std::string> names = {"X_WF", "X_WG"};
  //  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  //  pose1.translation() = Eigen::Vector3d::Ones()*0.5;
  //  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  //  Eigen::Vector3d translation2; translation2 << 1,2,3;
  //  pose1.translation() = translation2;
  //  std::vector<Eigen::Isometry3d> poses = {pose1, pose2};

  PublishFramesToLcm("DRAKE_DRAW_TRAJECTORY", poses, names, &lcm);
}
}  // namespace eve
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple dynamic simulation for the Allegro hand moving under constant"
      " torques.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::eve::DoMain();
  return 0;
}

```
{% endcode-tabs-item %}
{% endcode-tabs %}

{% code-tabs %}
{% code-tabs-item title="BUILD.bazel" %}
```text
drake_cc_binary(
    name = "trajectory_publisher",
    srcs = ["trajectory_publisher.cc"],
    data = [
        "//manipulation/models/eve:models",
        "//manipulation/models/qb_hand_description:models",
    ],
    deps = [
        "//common:find_resource",
        "//common:text_logging_gflags",
        "//geometry:geometry_visualization",
        "//lcm",
        "//lcmtypes:viewer",
        "//multibody/benchmarks/inclined_plane",
        "//multibody/parsing",
        "//multibody/plant",
        "//multibody/plant:contact_results_to_lcm",
        "//systems/analysis",
        "//systems/controllers",
        "//systems/primitives",
        "//systems/rendering:pose_bundle_to_draw_message",
        "@gflags",
    ],
)
```
{% endcode-tabs-item %}
{% endcode-tabs %}

### Run the demo

Compile the visualizer again, and open visualizer with plugin.

```text
bazel build //tools:drake_visualizer
bazel-bin/tools/drake_visualizer --script=tools/workspace/drake_visualizer/plugin/show_trajectory.py
```

Run the trajectory publisher.

```text
bazel run //examples/viz:trajectory_publisher
```

