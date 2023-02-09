# Create a URDF/SDF robot

To create a Multibody robot, read from a robot description file is a convenient way. Drake support parsing URDF and SDF files.

### Run the demo

{% hint style="info" %}
The code is also [available on my github](https://github.com/guzhaoyuan/drake/tree/tutorial/examples/hello), you could also [skip to the result](https://drake.guzhaoyuan.com/thing-to-do-in-drake/create-a-urdf-sdf-robot#quick-access-to-the-result).
{% endhint %}

#### Create workspace & get model

```
cd drake
mkdir -p examples/double_pendulum_pid/models
curl -o examples/double_pendulum_pid/models/double_pendulum.sdf https://raw.githubusercontent.com/guzhaoyuan/drake/tutorial/examples/double_pendulum_pid/models/double_pendulum.sdf
```

#### Create the source file

{% code title="BUILD.bazel" %}
```
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
{% endcode %}

{% code title="run_double_pendulum_passive.cc" %}
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
{% endcode %}

#### Run the code

```
bazel-bin/tools/drake_visualizer &
bazel run //examples/double_pendulum_pid:run_double_pendulum_passive_exe
```

### The Code Explained

{% code title="BUILD.bazel" %}
```
install_data()
```
{% endcode %}

The `install_data()` copy the items under the `data` tag to the build folder so when the program actually runs, it will be able to access the data files.

{% code title="run_double_pendulum_passive.cc" %}
```cpp
DRAKE_DEMAND(FLAGS_simulation_time > 0);
```
{% endcode %}

`DRAKE_DEMAND` is an assertion that ensures that the FLAGS\_simulation\_time is larger than zero. If not fulfilled, the program will stop throwing an exception. We could always use `DRAKE_DEMAND` to verify data correctness.&#x20;

{% code title="run_double_pendulum_passive.cc" %}
```cpp
systems::DiagramBuilder<double> builder;
```
{% endcode %}

Create a `DiagramBuilder` that helps to add `system` blocks and connect different systems.

{% code title="run_double_pendulum_passive.cc" %}
```cpp
geometry::SceneGraph<double>& scene_graph =
    *builder.AddSystem<geometry::SceneGraph>();
scene_graph.set_name("scene_graph");
```
{% endcode %}

Add a `SceneGraph<double>` to the `diagram` using `builder`. Note that `system` have we added was recorded and stored. The `diagram` and all the `system` are created when we call:

{% code title="run_double_pendulum_passive.cc" %}
```cpp
auto diagram = builder.Build();
```
{% endcode %}

{% code title="run_double_pendulum_passive.cc" %}
```cpp
// Load and parse double pendulum SDF from file into a tree.
multibody::MultibodyPlant<double>* dp =
  builder.AddSystem<multibody::MultibodyPlant<double>>(FLAGS_max_time_step);
dp->set_name("plant");
dp->RegisterAsSourceForSceneGraph(&scene_graph);
```
{% endcode %}

Create a `MultibodyPlant<double>` that represents the double pendulum. Then we set its name and connect it to `SceneGraph` so we could see the model later in simulation.

{% code title="run_double_pendulum_passive.cc" %}
```cpp
multibody::Parser parser(dp);
const std::string sdf_path = FindResourceOrThrow(kDoublePendulumSdfPath);
multibody::ModelInstanceIndex plant_model_instance_index =
  parser.AddModelFromFile(sdf_path);
(void)plant_model_instance_index;
```
{% endcode %}

Here we create a parser that actually create the model. It will parse the provided URDF or SDF file line by line, and call Drake functions internally to create a multibody in tree structure.

{% code title="run_double_pendulum_passive.cc" %}
```cpp
// Weld the base link to world frame with no rotation.
const auto& root_link = dp->GetBodyByName("base");
dp->AddJoint<multibody::WeldJoint>("weld_base", dp->world_body(), nullopt,
  root_link, nullopt,
  Isometry3<double>::Identity());
// Now the plant is complete.
dp->Finalize();  

DRAKE_DEMAND(!!dp->get_source_id());
```
{% endcode %}

We then create a fixed joint `WeldJoint` between the robot base and the world frame. So the robot base always stay fixed on the ground. Once the MultibodyPlant is finished, we call `dp->Finalize()` so we seal the plant, making sure the robot model is not mutable during simulation. The `DRAKE_DEMAND` makes sure that the MultibodyPlant is created successfully.

{% code title="run_double_pendulum_passive.cc" %}
```cpp
builder.Connect(
  dp->get_geometry_poses_output_port(),
  scene_graph.get_source_pose_port(dp->get_source_id().value()));
builder.Connect(scene_graph.get_query_output_port(),
                dp->get_geometry_query_input_port());

geometry::ConnectDrakeVisualizer(&builder, scene_graph);
```
{% endcode %}

After the plant is created. We use builder again to connect the `MultibodyPlant` system with other blocks in the `diagram`. `scene_graph` receive the `MultibodyPlant` current state and compute the collision in the system. It then reports all the collision infomation to the `MultibodyPlant` so `MultibodyPlant` will decide the contact force according to the contact body material.

{% code title="run_double_pendulum_passive.cc" %}
```cpp
auto diagram = builder.Build();
std::unique_ptr<systems::Context<double>> diagram_context =
  diagram->CreateDefaultContext();

// Create plant_context to set velocity.
systems::Context<double>& plant_context =
  diagram->GetMutableSubsystemContext(*dp, diagram_context.get());
```
{% endcode %}

Create the `diagram`  and `diagram_context`. Get the `plant_context` by getting subsystem context from the `diagram_context`.

{% code title="run_double_pendulum_passive.cc" %}
```cpp
// Set init position.
Eigen::VectorXd positions = Eigen::VectorXd::Zero(2);
positions[0] = 0.1;
positions[1] = 0.4;
dp->SetPositions(&plant_context, positions);
```
{% endcode %}

Set the initial position of the robot away from the home position.

{% code title="run_double_pendulum_passive.cc" %}
```cpp
systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
simulator.set_publish_every_time_step(true);
simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
simulator.Initialize();
simulator.AdvanceTo(FLAGS_simulation_time);
```
{% endcode %}

Create a `Simulator` , set the simulation parameter. Then actually do the simulation by advance to the desired time.

### Quick access to the result

Add code to you repository.

```
cd drake
git remote add gzy https://github.com/guzhaoyuan/drake.git
git pull gzy tutorial
git checkout tutorial
```

Run.

```
bazel-bin/tools/drake_visualizer &
bazel run //examples/double_pendulum_pid:run_double_pendulum_passive_exe
```

### Extensions

Similar example of parsing an Allegro hand.

```
bazel-bin/tools/drake_visualizer &
bazel run //examples/allegro_hand:run_allegro_constant_load_demo
```
