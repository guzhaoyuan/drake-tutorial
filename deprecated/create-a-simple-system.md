# Create a simple system

Before starting, make sure you understand the basic Drake concept at this link.

{% content-ref url="../introduction/drake-concept.md" %}
[drake-concept.md](../introduction/drake-concept.md)
{% endcontent-ref %}

### Goal

Given a Equation of Motion (EoM) of a particle mass system, create the exact system which moves with external force. Simulate the system and visualize the results.

### Analysis

A non-linear system has EoM like this:

$$
\dot{x} = f(t,x,u) \\
y = g(t,x,u)
$$

Say in our example, the system has EoM like this:

$$
\ddot{x} = f(t)/m \\
y(t) = x(t) \\
where,  x(t) = [x, \dot{x}]'
$$

The system get external force $$f(t)$$ from input port, then we compute the system state and write the result observation $$y(t)$$ to output port.

The $$f(t)$$ comes from a input source, the output of the system goes to the visualization so we could observe the result.

![](<../.gitbook/assets/Block diagram (1).png>)

### Code Implementation

#### Create the system

```cpp
class Particle1dPlant final : public systems::LeafSystem<T> {
 public:
  // Disables the built in copy and move constructor.
 DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Particle1dPlant);
 
 /// Constructor defines a prototype for its continuous state and output port.
 Particle1dPlant();
 
 /// Return the OutputPort associated with this Particle1dPlant.
 /// This method is called when connecting the ports in the diagram builder.
 const systems::OutputPort<T>& get_output_port() const {
    return systems::System<T>::get_output_port(0);
 }
 /// Returns the current state of this Particle1dPlant as a BasicVector.
 /// The return value is mutable to allow the calling method to change the
 /// state (e.g., to set initial values). For example, this method is called
 /// when building a diagram so initial values can be set by the simulator.
 /// @param[in] context The Particle1dPlant sub-system context.
 static systems::BasicVector<T>& get_mutable_state(
      systems::Context<T>* context) {
    return get_mutable_state(&context->get_mutable_continuous_state());
 }
 
 private:
  // Casts the continuous state vector from a VectorBase to a BasicVector.
 static const systems::BasicVector<T>& get_state(
      const systems::ContinuousState<T>& cstate) {
    return dynamic_cast<const systems::BasicVector<T>&>(cstate.get_vector());
 }
 
  // This method is called in DoCalcTimeDerivative() as a way to update the
 // state before the time derivatives are calculated.
 static const systems::BasicVector<T>& get_state(
      const systems::Context<T>& context) {
    return get_state(context.get_continuous_state());
 }
 
  // Casts the mutable continuous state vector from a VectorBase to BasicVector.
 static systems::BasicVector<T>& get_mutable_state(
      systems::ContinuousState<T>* cstate) {
    return dynamic_cast<systems::BasicVector<T>&>(cstate->get_mutable_vector());
 }
 
 // This is the calculator method that assigns values to the state output port.
 void CopyStateOut(const systems::Context<T>& context,
 systems::BasicVector<T>* output) const;
 
 // Method that calculates the state time derivatives.
 void DoCalcTimeDerivatives(const systems::Context<T>& context,
 systems::ContinuousState<T>* derivatives) const override;
};
```

#### Build the diagram

```cpp
// Parsing the URDF and constructing a RigidBodyTree from it
auto tree =
    std::make_unique<RigidBodyTree<double>>();
parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    FindResourceOrThrow("drake/examples/particle1d/particle1d.urdf"),
    multibody::joints::kFixed, tree.get());
 
// Construct and empty diagram, then add a particle plant.
systems::DiagramBuilder<double> builder;
 
Particle1dPlant<double>* particle_plant =
    builder.AddSystem<Particle1dPlant<double>>();
particle_plant->set_name("RigidBox");
 
// To set constant parameters (such as mass) in the particle plant, pass
// information from the tree (which read constant parameters in the .urdf).
particle_plant->SetConstantParameters(*tree);
 
// Add a visualizer block to the diagram, uses the tree and an lcm object.
lcm::DrakeLcm lcm;
systems::DrakeVisualizer* publisher =
    builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
publisher->set_name("publisher");
 
// Within the diagram, connect the particle plant's output port to the
// visualizer's input port.
builder.Connect(particle_plant->get_output_port(),
                publisher->get_input_port(0));
 
// Build the diagram described by previous call to connect input/export port.
auto diagram = builder.Build();
 
// Constructs a Simulator that advances this diagram through time.
systems::Simulator<double> simulator(*diagram);
 
// To set initial values for the simulation:
// * Get the Diagram's context.
// * Get the part of the Diagram's context associated with particle_plant.
// * Get the part of the particle_plant's context associated with state.
// * Fill the state with initial values.
systems::Context<double>& simulator_context = simulator.get_mutable_context();
systems::Context<double>& particle_plant_context =
    diagram->GetMutableSubsystemContext(*particle_plant, &simulator_context);
systems::BasicVector<double>& state =
    particle_plant->get_mutable_state(&particle_plant_context);
 
const double x_init = 0.0;
const double xDt_init = 0.0;
state.SetAtIndex(0, x_init);
state.SetAtIndex(1, xDt_init);
 
// Set upper-limit on simulation speed (mostly for visualization).
simulator.set_target_realtime_rate(1);
 
// Simulate for 10 seconds (default units are SI, with units of seconds).
std::cout << "Starting simulation." << std::endl;
simulator.StepTo(10.0);
std::cout << "Simulation done." << std::endl;
```

### Credit

Thanks Manuel Ahumada for providing this example.
