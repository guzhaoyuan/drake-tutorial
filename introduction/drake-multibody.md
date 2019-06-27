# Drake Multibody

Multibody is the highlight of Drake. It represents a robot of a Multibody tree, and exposes APIs that computes robot mathematics including kinematics, dynamics, jacobian, etc.  
A very good material to start read is [Drake Documentation about Multibody](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html#details). It show how Multibody is constructed and how it fit with other drake components.

### Robot Modeling

#### Create Multibody

To create a Multibody robot, the best way is to parse robot description file like URDF or SDF. Drake could read URDF or SDF using a `parser`. The `parser` use [tinyxml2](https://github.com/leethomason/tinyxml2) library to parse the XML file. It would iteratively parse each link, joint, transmission and generate a Multibody tree structure as described in the XML file.

Another way is to create the robot components using Drake API. For example, we could create [bodies](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_body.html) and connect bodies with [joints](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_joint.html). This normally works for simple model like inverted pendulum.

#### Be ware of some Drake feature

Be careful that Drake does not support mesh file for geometry collision yet. In other words, if you need to simulate collision, you would need to create your own collision model, usually some simple geometry that wraps around your mesh file. \(I found it more convenient to add collision model using Rviz, because it has a check box allows you to show the collision geometry, way easier to compare between mesh and the simple collision geometry.\)

If you migrate your robot from ROS, your URDF is probably using package to find the mesh file rather than using relative path. Drake support "package" which is used in ROS. However, you will at least need to change the folder with URDF files into a `package`, which means you should provide a `package.xml` file and modify the `BUILD.bazel` file to make the `bazel` build system recognize the `package`. In addition, in your code, the `package.xml` file path should be specified, to make the program recognize the package as well. There is a example for that. \(TODO: Add example here.\)

#### Default behavior Drake would do while parsing

One thing worth mentioning is that drake requires the model to have at least one `transmission` in the URDF, or it will fail to create input torque port for the plant. So if you do not have any `transmission` tag in the URDF, do create `SimpleTransmission` for each actuated joint.

One more thing, every model created in drake by default is a floating base plant, with a default `world_frame` and a floating base joint connected to the `world_frame`. So if your robot should be fixed to the ground, you would create new joints in drake to specify what kind of joint the robot base is connected with the world.

#### [Mobilizer](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1internal_1_1_mobilizer.html)

Drake uses `Mobilizer` to connect two links, it is a different concept from the traditional `Joint`.

### Kinematics

After the URDF file is imported, the `MultibodyPlant` is created. The plant is a class hosting a whole bunch of useful robotics function that we use to do motion planning and control.

### Jacobian

Drake support compute all kinds of jacobian. To calculate the derivative with respect to something, `AutoDiff` is used to ease the way of calculating.

### Dynamics

Drake support compute all kinds of dynamics algorithm, especially the ones in the book of [Roy Featherstone 2008](https://link.springer.com/book/10.1007/978-1-4899-7560-7).  


