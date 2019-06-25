# Visualize trajectory in Drake Visualizer

Create a plugin in drake/tools/workspace/drake\_visualizer/plugin/show\_trajectory.py

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

Added publisher cpp code and BUILD.bazel under _drake/examples/viz_.

{% code-tabs %}
{% code-tabs-item title="trajectory\_publisher.cc" %}
```cpp
/// @file
///
/// This file create a simple linear trajectory and make a PID controller to
/// follow the trajectory.

/**
Example usage:
DrakeLcm lcm;
PublishFramesToLcm("DRAKE_DRAW_FRAMES", {
    {"X_WF", Isometry3d::Identity()},
    {"X_WG", Isometry3d::Identity()},
});
 */
#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/lcmt_viewer_draw.hpp"


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
  dlcm->Publish(
      "DRAKE_DRAW_TRAJECTORY_" + channel_name, bytes.data(), num_bytes, {});
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
  lcm::DrakeLcm lcm;
  PublishFramesToLcm("DRAKE_DRAW_TRAJECTORY", {
      {"X_WF", Eigen::Isometry3d::Identity()},
      {"X_WG", Eigen::Isometry3d::Identity()},
  }, &lcm);
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

/// @file
///
/// This file create a simple linear trajectory and make a PID controller to
/// follow the trajectory.

/**
Example usage:
DrakeLcm lcm;
PublishFramesToLcm("DRAKE_DRAW_FRAMES", {
    {"X_WF", Isometry3d::Identity()},
    {"X_WG", Isometry3d::Identity()},
});
 */
#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/lcmt_viewer_draw.hpp"


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
  dlcm->Publish(
      "DRAKE_DRAW_TRAJECTORY_" + channel_name, bytes.data(), num_bytes, {});
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
  lcm::DrakeLcm lcm;
  PublishFramesToLcm("DRAKE_DRAW_TRAJECTORY", {
      {"X_WF", Eigen::Isometry3d::Identity()},
      {"X_WG", Eigen::Isometry3d::Identity()},
  }, &lcm);
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

