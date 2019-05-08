# aurova_planning
This is a metapackage that contains different packages that perform global planning algorithims. Compiling this metapackage into ROS will compile all the packages at once. This metapackage is grouped as a project for eclipse C++. Each package contains a "packagename_doxygen_config" configuration file for generate doxygen documentation. The packages contained in this metapackage are:

**get_pose_from_tf**
This package contains a node that, as input, reads the /tf messages. This node calculates the transformation between two differents frames to obtain a odometry message. The node output is published in the topic /odometry_filtered of type nav_msgs::Odometry.
* ~frame_id_tf (default: ""): Parent frame name to transform.
* ~child_id_tf (default: ""): Child frame name to transform.

**path_planning_demo**
