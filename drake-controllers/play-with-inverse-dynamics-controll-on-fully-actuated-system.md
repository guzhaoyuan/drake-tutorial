# Inverse Dynamics Control on fully-actuated KUKA arm

### Code

Create a folder _drake/examples/simple\_arm._

```text
cd drake
mkdir -p examples/simple_arm
```

Put the following file into the folder.

{% code-tabs %}
{% code-tabs-item title="run\_simple\_arm.cc" %}
```cpp
/// @file
///
/// This demo sets up a simple arm simulation using the multi-body library.
/// A inverse dynamics controller was created to control the robot around
/// the desired system states.

#include <gflags/gflags.h>

#include <drake/common/type_safe_index.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace simple_arm {

using drake::multibody::MultibodyPlant;

DEFINE_double(constant_load, 0,
              "the constant load on each joint, Unit [Nm]."
              "Suggested load is in the order of 0.01 Nm. When input value"
              "equals to 0 (default), the program runs a passive simulation.");

DEFINE_double(simulation_time, 5,
              "Desired duration of the simulation in seconds");

DEFINE_bool(use_right_hand, true,
            "Which hand to model: true for right hand or false for left hand");

DEFINE_double(max_time_step, 1.0e-3,
              "Simulation time step used for integrator.");

DEFINE_bool(add_gravity, true,
            "Indicator for whether terrestrial gravity"
            " (9.81 m/s²) is included or not.");

DEFINE_double(target_realtime_rate, 1,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

void DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  systems::DiagramBuilder<double> builder;

  geometry::SceneGraph<double>& scene_graph =
      *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double>& plant =
      *builder.AddSystem<MultibodyPlant>(FLAGS_max_time_step);
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  std::string full_name = FindResourceOrThrow(
      "drake/manipulation/models/"
      "iiwa_description/sdf/iiwa14_no_collision.sdf");
  // "simple_arm/model.sdf");

  multibody::ModelInstanceIndex plant_index =
      multibody::Parser(&plant).AddModelFromFile(full_name);

  // Weld the hand to the world frame
  const auto& joint_arm_root = plant.GetBodyByName("iiwa_link_0");
  plant.AddJoint<multibody::WeldJoint>("weld_arm", plant.world_body(), nullopt,
                                       joint_arm_root, nullopt,
                                       Isometry3<double>::Identity());

  if (!FLAGS_add_gravity) {
    plant.mutable_gravity_field().set_gravity_vector(Eigen::Vector3d::Zero());
  }

  // Now the model is complete.
  plant.Finalize();

  const int U = plant.num_actuators();
  // constant force input
  VectorX<double> constant_load_value =
      VectorX<double>::Ones(U * 2) * FLAGS_constant_load;
  auto constant_source =
      builder.AddSystem<systems::ConstantVectorSource<double>>(
          constant_load_value);
  constant_source->set_name("constant_source");

  drake::log()->info("positions " + std::to_string(plant.num_positions()) +
                     ", velocities " + std::to_string(plant.num_velocities()) +
                     ", actuators " + std::to_string(plant.num_actuators()));
  auto IDC =
      builder
          .AddSystem<systems::controllers::InverseDynamicsController<double>>(
              plant, Eigen::VectorXd::Ones(U) * 1000.0,
              Eigen::VectorXd::Ones(U) * 0.1, Eigen::VectorXd::Ones(U) * 1.0,
              false);
  builder.Connect(IDC->get_output_port_control(),
                  plant.get_actuation_input_port());
  builder.Connect(plant.get_state_output_port(),
                  IDC->get_input_port_estimated_state());
  builder.Connect(constant_source->get_output_port(),
                  IDC->get_input_port_desired_state());

  DRAKE_DEMAND(!!plant.get_source_id());
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());

  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());
  Eigen::VectorXd q = plant.GetPositions(plant_context, plant_index);
  q[0] = 0.1;
  q[1] = 0.3;
  q[2] = 0.3;
  plant.SetPositions(&plant_context, q);

  // Set up simulator.
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);
}

}  // namespace simple_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple dynamic simulation for the Allegro hand moving under constant"
      " torques.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::simple_arm::DoMain();
  return 0;
}
```
{% endcode-tabs-item %}
{% endcode-tabs %}

Added BUILD.bazel

{% code-tabs %}
{% code-tabs-item title="BUILD.bazel" %}
```text
load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools/lint:lint.bzl", "add_lint_tests")

package(
    default_visibility = [":__subpackages__"],
)

drake_cc_binary(
    name = "run_simple_arm",
    srcs = ["run_simple_arm.cc"],
    data = [
        "//manipulation/models/iiwa_description:models",
    ],
    deps = [
        "//common:find_resource",
        "//common:text_logging_gflags",
        "//common:type_safe_index",
        "//geometry:geometry_visualization",
        "//lcm",
        "//multibody/parsing",
        "//multibody/plant",
        "//systems/analysis",
        "//systems/controllers",
        "//systems/primitives:constant_vector_source",
        "//systems/rendering:pose_bundle_to_draw_message",
        "@gflags",
    ],
)


add_lint_tests()
```
{% endcode-tabs-item %}
{% endcode-tabs %}

### Run demo

```text
bazel run //examples/simple_arm:run_simple_arm
```

