# Drake Multibody

Multibody is the key part of Drake. It represents the robot itself with a Multibody tree, and exposes APIs that help to compute robot mathematics including kinematics, dynamics, jacobian, etc.  
A very good material to start read is [Drake Documentation about Multibody](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html#details). It show how Multibody is constructed and how it fit with other drake components.

### Robot Modeling

Drake could read URDF or SDF using a parser. The parser use [tinyxml2](https://github.com/leethomason/tinyxml2) library to parse the xml file. It would parse each link, joint, transmission in the xml file.

The downside of drake is it does not support mesh file geometry for collision. In other words, if you need to simulate collision, you would need to create your own collision model, usually some simple geometry that wraps around your mesh file. \(I found it more convenient to add collision model using Rviz, because it has a check box so you could choose to show the collision geometry or not, way easier to compare between mesh and collision geometry than drake.\)

Drake do support the similar package mechanism in ROS, so if your URDF is referring the mesh file from package rather than relative path, drake got you covered. However, you will need to make the folder hosting URDF file a `package`, which means you will need to provide the `package.xml` file and change the `BUILD.bazel` file to make the `bazel` build system recognize the package. In addition, you will have to allocate the path for the `package.xml` file to make the program could recognize the package as well. \(TODO: Add example here.\)

#### Some default behavior Drake would do

One thing worth mentioning is that drake requires the model to have at least one `transmission` in the URDF, or it will fail to create input torque port for the plant. So if you do not have any `transmission` tag in the URDF, do create `SimpleTransmission` for each actuated joint.

One more thing, every model created in drake by default is a floating base plant, with a default `world_frame` and a floating base joint connected to the `world_frame`. So if your robot should be fixed to the ground, you would create new joints in drake to specify what kind of joint the robot base is connected with the world.

#### Mobilizer

Drake uses `Mobilizer` to connect two links, it is a different concept from the traditional `Joint`.

### Kinematics

After the URDF file is imported, the multibodyplant is created. The plant is a class hosting a whole bunch of useful robotics function that we use to do motion planning and control.

### Jacobian

Drake support compute all kinds of jacobian. To calculate the derivative wrt to something, `AutoDiff` is used to ease the way of calculating.

### Dynamics

Drake support compute all kinds of dynamics algorithm, especially the ones in the book of [Roy Featherstone 2008](https://link.springer.com/book/10.1007/978-1-4899-7560-7).  


