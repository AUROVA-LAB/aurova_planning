# aurova_planning
This is a metapackage that contains different packages that perform global planning algorithims. Compiling this metapackage into ROS will compile all the packages at once. This metapackage is grouped as a project for eclipse C++. Each package contains a "packagename_doxygen_config" configuration file for generate doxygen documentation. The packages contained in this metapackage are:

**get_pose_from_tf**
This package contains a node that, as input, reads the /tf messages. This node calculates the transformation between two differents frames to obtain a odometry message. The node output is published in the topic /odometry_filtered of type nav_msgs::Odometry.
* ~frame_id_tf (default: ""): Parent frame name to transform.
* ~child_id_tf (default: ""): Child frame name to transform.

**path_planning_demo**
This package contains a node that, as input, reads the topics /request_goal of type std_msgs::Bool, /re_locate of type std_msgs::Bool, and /amcl_pose of type geometry_msgs::PoseWithCovariance. The node output is published in the topics /visualization of type visualization_msgs::MarkerArray, and /move_base_simple/goal of type geometry_msgs::PoseStamped.
* ~mode_path (default: 0): 1 -> close loop, 2 -> random trajectories, 3 -> global goal.
* ~frame_id_markers (default: ""): Frame where the markers representing the trajectory will be published..
* ~path_file_links (default: ""): Path to graph links generated with matlab scripts privided in this package.
* ~path_file_nodes (default: ""): Path to graph nodes generated with matlab scripts privided in this package.
* ~path_file_goals (default: ""): Path to graph goals generated with matlab scripts privided in this package.
