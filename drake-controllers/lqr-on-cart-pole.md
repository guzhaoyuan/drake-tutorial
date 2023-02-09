# LQR on Cart Pole

Linear Quadratic Regulator (LQR) is an optimal control method. Cart-Pole is a canonical model with one prismatic joint connecting the ground and cart and one revolute joint connect the cart and bar.

![Cart-Pole tracking state](../.gitbook/assets/cart\_pole\_tracking.gif)

### A brief explanation of LQR

LQR controller is a gain matrix K, which maps income system state to control. The gain matrix K comes from the solution of Riccati Equation.&#x20;

Optimal control is all about minimizing cost. We formulate the state tracking control problem into a cost minimizing problem, which is equal to solve the Riccati Equation. What's amazing is, the solution of Riccati Equation gives us the optimal controller that generates the minimum cost.

So given the system and cost definition, we could compute the optimal controller K.

To learn more about Cart-Pole model and LQR, find the notes from [Russ Tedrake's Underactauted Robotics](http://underactuated.mit.edu/underactuated.html?chapter=acrobot).

### Get the code

Get the code from my repo, we need to modify the original SDF file to get a fully actuated Cart Pole. And we need a fixed DARE solver in Drake.

```markup
<sdf version='1.6'>
  <model name='CartPole'>
    <!-- This sdf file produces a model with the default parameters as
         documented in cart_pole_params.named_vector.
         They MUST be kept in sync. -->
    <link name='Cart'>
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>10.0</mass>
        <!-- For this model case, with the cart not having any rotational
             degrees of freedom, the values of the inertia matrix do not
             participate in the model. Therefore we just set them to zero
             (or near to zero since sdformat does not allow exact zeroes
             for inertia values). -->
        <inertia>
          <ixx>1.0e-20</ixx><iyy>1.0e-20</iyy><izz>1.0e-20</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>

      <visual name='cart_visual'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>0.24 0.12 0.12</size>
          </box>
        </geometry>
      </visual>
    </link>

    <link name='Pole'>
      <!-- The pole is modeled as a point mass at the end of a pole. -->
      <!-- The length of the pole is 0.5 meters. -->
      <pose>0 0 -0.5 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <!-- A point mass has zero rotational inertia.
             We must specify small values since otherwise sdformat throws an
             exception. -->
        <inertia>
          <ixx>1.0e-20</ixx><iyy>1.0e-20</iyy><izz>1.0e-20</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>

      <visual name='pole_point_mass'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>0.05</radius>
          </sphere>
        </geometry>
      </visual>
      <visual name='pole_rod'>
        <pose>0 0 0.25 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.025</radius>
            <length>0.5</length>
          </cylinder>
        </geometry>
      </visual>
    </link>

    <joint name='CartSlider' type='prismatic'>
      <parent>world</parent>
      <child>Cart</child>
      <axis>
        <xyz>1.0 0.0 0.0</xyz>
        <limit>
          <!-- The pole pin joint is not actuated. -->
          <effort>1.0</effort>
        </limit>
      </axis>
    </joint>

    <joint name='PolePin' type='revolute'>
      <!-- Pose of the joint frame in the pole's frame (located at the point
           mass) -->
      <pose>0 0 0.5 0 0 0</pose>
      <parent>Cart</parent>
      <child>Pole</child>
      <axis>
        <xyz>0.0 -1.0 0.0</xyz>
        <limit>
          <!-- The pole pin joint is not actuated. -->
          <effort>1.0</effort>
        </limit>
      </axis>
    </joint>
  </model>
</sdf>
```

{% code title="cart_pole_lqr.cc" %}
```cpp
///
/// This file use a fully actuated cart pole model to track a specific state
///

#include <gflags/gflags.h>

#include <drake/systems/framework/event.h>
#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/linear_system.h"

DEFINE_double(target_realtime_rate, 1.0,
              "Rate at which to run the simulation, relative to realtime");
DEFINE_double(simulation_time, 10, "How long to simulate the pendulum");
DEFINE_double(max_time_step, 1.0e-3,
              "Simulation time step used for integrator.");

namespace drake {
namespace examples {
namespace multibody {
namespace cart_pole {
namespace {

// Fixed path to double pendulum SDF model.
static const char* const kCartPoleSdfPath =
    "drake/examples/multibody/cart_pole/cart_pole.sdf";

//
// Main function for demo.
//
void DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_time > 0);
  logging::HandleSpdlogGflags();

  systems::DiagramBuilder<double> builder;

  geometry::SceneGraph<double>& scene_graph =
      *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Load and parse double pendulum SDF from file into a tree.
  drake::multibody::MultibodyPlant<double>* cp =
      builder.AddSystem<drake::multibody::MultibodyPlant<double>>(
          FLAGS_max_time_step);
  cp->set_name("cart_pole");
  cp->RegisterAsSourceForSceneGraph(&scene_graph);

  drake::multibody::Parser parser(cp);
  const std::string sdf_path = FindResourceOrThrow(kCartPoleSdfPath);
  drake::multibody::ModelInstanceIndex plant_model_instance_index =
      parser.AddModelFromFile(sdf_path);
  (void)plant_model_instance_index;

  // Now the plant is complete.
  cp->Finalize();

  // Create LQR Controller.
  auto cp_context = cp->CreateDefaultContext();
  const int CartPole_actuation_port = 3;
  // Set nominal torque to zero.
  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(2);
  cp_context->FixInputPort(CartPole_actuation_port, u0);

  // Set nominal state to the upright fixed point.
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
  x0[0] = 1;
  x0[1] = M_PI;
  cp_context->SetDiscreteState(x0);

  // Setup LQR Cost matrices (penalize position error 10x more than velocity
  // to roughly address difference in units, using sqrt(g/l) as the time
  // constant.
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(4, 4);
  Q(0, 0) = 10;
  Q(1, 1) = 10;
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);
  Eigen::MatrixXd N;
  auto lqr = builder.AddSystem(systems::controllers::LinearQuadraticRegulator(
      *cp, *cp_context, Q, R, N, CartPole_actuation_port));

  builder.Connect(cp->get_state_output_port(), lqr->get_input_port());
  builder.Connect(lqr->get_output_port(), cp->get_actuation_input_port());

  // Connect plant with scene_graph to get collision information.
  DRAKE_DEMAND(!!cp->get_source_id());
  builder.Connect(
      cp->get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(cp->get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  cp->get_geometry_query_input_port());

  geometry::ConnectDrakeVisualizer(&builder, scene_graph);

  auto diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  // Create plant_context to set velocity.
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(*cp, diagram_context.get());
  // Set init position.
  Eigen::VectorXd positions = Eigen::VectorXd::Zero(2);
  positions[0] = 0.0;
  positions[1] = 0.0;
  cp->SetPositions(&plant_context, positions);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);
}

}  // namespace
}  // namespace cart_pole
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char** argv) {
  gflags::SetUsageMessage(
      "Using LQR controller to control "
      "the fully actuated cart pole model!");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::multibody::cart_pole::DoMain();
  return 0;
}
```
{% endcode %}

Add the following lines to `BUILD.bazel`.

{% code title="BUILD.bazel" %}
```
drake_cc_binary(
    name = "cart_pole_lqr",
    srcs = ["cart_pole_lqr.cc"],
    data = ["cart_pole.sdf"],
    deps = [
        "//common:find_resource",
        "//common:text_logging_gflags",
        "//geometry:geometry_visualization",
        "//multibody/parsing",
        "//multibody/plant",
        "//systems/analysis:simulator",
        "//systems/controllers",
        "//systems/framework:diagram",
        "//systems/framework:event_collection",
        "//systems/primitives",
        "@gflags",
    ],
)
```
{% endcode %}

### Run the demo

To get the demo working, a DARE solver is required. We could get the solver by:

```
git remote add weiqiao https://github.com/weiqiao/drake.git
git fetch weiqiao
git cherry-pick e777b31f04ec1d176a33d018669f62e9d3924e72
git cherry-pick aee6342eb01b142f5f109c6dba8eafa8f5994601
git cherry-pick a6f90a80f6842a67fe596c748e5b71eb56a7500a
```

Then run the demo:

```
bazel run //examples/multibody/cart_pole:cart_pole_lqr
```

### The code explained

### Implementation notes

LQR Controller in Drake is an affine system. Because LQR is about error dynamics, so the gain matrix K of LQR has to get the state error, and output the control error. With affine system, LQR could get state directly and output control directly, the conversion to state error and convert back to control is handled within the affine system.

Indexing is very important in Drake. A correct indexing is required to get the system work correctly. LQR is using Drake Index on control rather than generalized tree index.
