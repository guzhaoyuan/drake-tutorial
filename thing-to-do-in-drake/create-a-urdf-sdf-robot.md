# Create a URDF/SDF robot

To create a Multibody robot, read from a robot description file is a convenient way. Drake support parsing URDF and SDF files.

### Run the demo

{% hint style="info" %}
The code is also [available on my github](https://github.com/guzhaoyuan/drake/tree/tutorial/examples/hello), you could also [skip to the result](https://drake.guzhaoyuan.com/thing-to-do-in-drake/create-a-urdf-sdf-robot#quick-access-to-the-result).
{% endhint %}

#### Create workspace & get model

```text
cd drake
mkdir -p examples/double_pendulum_pid/models
curl -o examples/double_pendulum_pid/models/double_pendulum.sdf https://raw.githubusercontent.com/guzhaoyuan/drake/tutorial/examples/double_pendulum_pid/models/double_pendulum.sdf
```

#### BUILD.bazel

```text
load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_binary",
)
load("//tools/install:install_data.bzl", "install_data")

drake_cc_binary(
    name = "run_double_pendulum_passive_exe",
    srcs = [
        "run_double_pendulum_passive.cc",
    ],
    data = [
        ":models",
        "//tools:drake_visualizer",
    ],
    deps = [
        "//common:find_resource",
        "//common:text_logging_gflags",
        "//geometry:geometry_visualization",
        "//lcm",
        "//multibody/parsing",
        "//multibody/plant",
        "//systems/analysis",
        "//systems/framework",
        "@sdformat",
    ],
)

install_data()
```

#### run\_double\_pendulum\_passive.cc

{% code-tabs %}
{% code-tabs-item title="run\_double\_pendulum\_passive.cc" %}
```cpp
///
/// @brief  An SDF based double pendulum example.
///

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_double(target_realtime_rate, 1.0,
              "Rate at which to run the simulation, relative to realtime");
DEFINE_double(simulation_time, 10, "How long to simulate the pendulum");
DEFINE_double(max_time_step, 1.0e-3,
              "Simulation time step used for integrator.");

namespace drake {
namespace examples {
namespace double_pendulum {
namespace {

// Fixed path to double pendulum SDF model.
static const char* const kDoublePendulumSdfPath =
    "drake/examples/double_pendulum_pid/models/double_pendulum.sdf";

//
// Main function for demo.
//
void DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  systems::DiagramBuilder<double> builder;

  geometry::SceneGraph<double>& scene_graph =
      *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Load and parse double pendulum SDF from file into a tree.
  multibody::MultibodyPlant<double>* dp =
      builder.AddSystem<multibody::MultibodyPlant<double>>(FLAGS_max_time_step);
  dp->set_name("plant");
  dp->RegisterAsSourceForSceneGraph(&scene_graph);

  multibody::Parser parser(dp);
  const std::string sdf_path = FindResourceOrThrow(kDoublePendulumSdfPath);
  multibody::ModelInstanceIndex plant_model_instance_index =
      parser.AddModelFromFile(sdf_path);
  (void)plant_model_instance_index;

  // Weld the base link to world frame with no rotation.
  const auto& root_link = dp->GetBodyByName("base");
  dp->AddJoint<multibody::WeldJoint>("weld_base", dp->world_body(), nullopt,
                                     root_link, nullopt,
                                     Isometry3<double>::Identity());
  // Now the plant is complete.
  dp->Finalize();

  // Connect plant with scene_graph to get collision information.
  DRAKE_DEMAND(!!dp->get_source_id());
  builder.Connect(
      dp->get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(dp->get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  dp->get_geometry_query_input_port());

  geometry::ConnectDrakeVisualizer(&builder, scene_graph);

  auto diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  // Create plant_context to set velocity.
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(*dp, diagram_context.get());

  // Set init position.
  Eigen::VectorXd positions = Eigen::VectorXd::Zero(2);
  positions[0] = 0.1;
  positions[1] = 0.4;
  dp->SetPositions(&plant_context, positions);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);
}

}  // namespace
}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake

int main(int argc, char** argv) {
  gflags::SetUsageMessage(
      "bazel run "
      "//examples/double_pendulum_pid:run_double_pendulum_passive_exe");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::double_pendulum::DoMain();
  return 0;
}
```
{% endcode-tabs-item %}
{% endcode-tabs %}

#### Run the code

```text
bazel-bin/tools/drake_visualizer &
bazel run //examples/double_pendulum_pid:run_double_pendulum_passive_exe
```

### The Code Explained



### Quick access to the result

Added code to you repository.

```text
cd drake
git remote add gzy https://github.com/guzhaoyuan/drake.git
git pull gzy tutorial
git checkout tutorial
```

Run.

```text
bazel-bin/tools/drake_visualizer &
bazel run //examples/double_pendulum_pid:run_double_pendulum_passive_exe
```

### Links

The full code is [here](https://github.com/guzhaoyuan/drake/tree/tutorial/examples/double_pendulum_pid).

Similar example of parsing an Allegro hand.

```text
bazel-bin/tools/drake_visualizer &
bazel run //examples/allegro_hand:run_allegro_constant_load_demo
```

