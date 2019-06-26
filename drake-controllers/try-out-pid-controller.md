# PID control of double pendulum

We use a double pendulum URDF as model. The task of the PID controller is to do state feedback control and keep the double pendulum at the top. To get a sense of how the model look like, try the command:

```bash
bazel run //examples/double_pendulum:double_pendulum_demo
```

{% hint style="info" %}
The code in the tutorial is also available on my github, so you could also checkout my code directly if you get stuck.
{% endhint %}

In this tutorial, we will go through:

1. Create workspace and compile.
2. Import the double pendulum URDF as a `MultibodyPlant`.
3. Create a PID controller, and connect it to `MultibodyPlant`.
4. Simulate and visualize.

### Run the demo

#### Create workspace

Create workspace and files.

```text
cd drake
mkdir -p examples/double_pendulum_pid
touch BUILD.bazel
touch run_double_pendulum_pid.cc
```

#### Add BUILD.bazel

To make the work space compile, we add the following to `BUILD.bazel`

{% code-tabs %}
{% code-tabs-item title="BUILD.bazel" %}
```text
load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_binary",
)

load("//tools/install:install_data.bzl", "install_data")

drake_cc_binary(
    name = "run_double_pendulum_pid_exe",
    srcs = [
        "run_double_pendulum_pid.cc",
    ],
    data = [
        ":models",
        "//tools:drake_visualizer",
    ],
    deps = [
        "//common:essential",
        "//common:find_resource",
        "//common:text_logging_gflags",
        "//geometry:geometry_visualization",
        "//lcm",
        "//multibody/parsing",
        "//multibody/plant",
        "//systems/analysis",
        "//systems/controllers",
        "//systems/framework",
        "@sdformat",
    ],
)

install_data()
```
{% endcode-tabs-item %}
{% endcode-tabs %}

#### Add source file

And add the _run\_double\_pendulum\_pid.cc_ file

{% code-tabs %}
{% code-tabs-item title="run\_double\_pendulum\_pid.cc" %}
```cpp
///
/// @brief  An SDF based double pendulum example.
///

#include <gflags/gflags.h>

#include <drake/systems/controllers/pid_controlled_system.h>
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
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(target_realtime_rate, 1.0,
              "Rate at which to run the simulation, relative to realtime");
DEFINE_double(simulation_time, 10, "How long to simulate the pendulum");
DEFINE_double(max_time_step, 1.0e-3,
              "Simulation time step used for integrator.");
DEFINE_double(Kp_, 10.0, "Kp");
DEFINE_double(Ki_, 0.0, "Kp");
DEFINE_double(Kd_, 0.0, "Kp");

namespace drake {
namespace examples {
namespace double_pendulum {
namespace {

// Fixed path to double pendulum SDF model.
static const char* const kDoublePendulumSdfPath =
    "drake/examples/double_pendulum/models/double_pendulum.sdf";

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
  dp->AddJointActuator("a1", dp->GetJointByName("upper_joint"));
  dp->AddJointActuator("a2", dp->GetJointByName("lower_joint"));

  // Now the plant is complete.
  dp->Finalize();

  // Create PID Controller.
  const Eigen::VectorXd Kp =
      Eigen::VectorXd::Ones(dp->num_positions()) * FLAGS_Kp_;
  const Eigen::VectorXd Ki =
      Eigen::VectorXd::Ones(dp->num_positions()) * FLAGS_Ki_;
  const Eigen::VectorXd Kd =
      Eigen::VectorXd::Ones(dp->num_positions()) * FLAGS_Kd_;
  const auto* const pid =
      builder.AddSystem<systems::controllers::PidController<double>>(Kp, Ki,
                                                                     Kd);
  builder.Connect(dp->get_state_output_port(),
                  pid->get_input_port_estimated_state());
  builder.Connect(pid->get_output_port_control(),
                  dp->get_actuation_input_port());
  // Set PID desired states.
  auto desired_base_source =
      builder.AddSystem<systems::ConstantVectorSource<double>>(
          Eigen::VectorXd::Zero(dp->num_multibody_states()));
  builder.Connect(desired_base_source->get_output_port(),
                  pid->get_input_port_desired_state());

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
      "Simple sdformat usage example, just"
      "make sure drake-visualizer is running!");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::double_pendulum::DoMain();
  return 0;
}
```
{% endcode-tabs-item %}
{% endcode-tabs %}

#### Simulate and visualize

```text
bazel-bin/tools/drake_visualizer &
bazel run //examples/double_pendulum_pid:run_double_pendulum_pid_exe -- --Kp_=1000 --Ki_=5 --Kd_=100
```

### Try with your own model

You could bring your own simple URDF/SDF \(with 1 or 2 degree of freedom\) and try doing a position control with the PID controller.

