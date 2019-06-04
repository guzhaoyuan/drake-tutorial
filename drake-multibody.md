# Drake Multibody

Multibody is the library that has the most related to me. I need to figure out all the features drake has or does not have to make full use of the library.  


### Robot Modelling

Drake could read urdf or sdf using a parser.

The downside is the drake does not support the mesh file for collision.

Drake do support the package mechanism in ROS, so if your urdf is refering the mesh file from package rather than relative path, drake got you covered. However, you will need to make the folder hosting urdf file a `package`, which means you will need to provide the `package.xml` file and change the `BUILD.bazel` file to make the bazel build system recognize the package. In addition, you will have to allocate the path for the `package.xml` file to make the program could recognize the package as well.  


The parser use [tinyxml2](https://github.com/leethomason/tinyxml2) library to parse the urdf xml file. It would parse the link, joint, transmission, etc.

One thing need to mention is that drake requires you have at least one transmission in the urdf, or it will fail to create a input torque port for the plant. So if you do not have any `transmission` tag in the urdf, do create at least a `SimpleTransmission`.  


### Kinematics

After the URDF file is imported, the multibodyplant is created. The plant is a class hosting a whole bunch of useful robotics function that we use to do motion planning and control.  


### Jacobian

Drake support compute all kinds of jacobian. To calculate the derivative wrt to something, `AutoDiff` is used to ease the way of calculating.  


### Dynamics

Drake support compute all kinds of dynamics algorithm, especially the ones in the book of [Roy Featherstone 2008](https://link.springer.com/book/10.1007/978-1-4899-7560-7).  


