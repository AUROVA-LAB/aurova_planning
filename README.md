# aurova_planning
This is a metapackage that contains different packages that perform global planning algorithims. Compiling this metapackage into ROS will compile all the packages at once. This metapackage is grouped as a project for eclipse C++. Each package contains a "packagename_doxygen_config" configuration file for generate doxygen documentation. The packages contained in this metapackage are:

**global_planning**

Parameters:
* ~global_planning/url_path (default: ""): Path to OSM file for graph representation.
* ~global_planning/frame_id (default: ""): Coorditates frame for planning (usually "map").
* ~global_planning/var_x (default: null): Variance in x axis.
* ~global_planning/var_y (default: null): Variance in y axis.
* ~global_planning/var_z (default: null): Variance in z axis.
* ~global_planning/var_w (default: null): Variance in yaw component.
* ~global_planning/type_dist (default: ""): Use "E" for euclidean distance, and "M" for Mahalanobis distance.
* ~global_planning/rad_reached (default: null): Radius to consider a goal reached.
* ~global_planning/stop_code (default: null): Define code to warn that the final goal was reached.
* ~global_planning/operation_mode (default: null): Define operation mode: 1) follow path to global goal. 2) follow closed loop 3) bypass global goal.
* ~global_planning/closed_loop (default: null): If operation_mode is 2, define vector with closed-loop nodes. 
* ~global_planning/save_data (default: false): If it is true, the data will save in a file.
* ~global_planning/url_file_out (default: ""): If save_data is true, define here de path of output data.

**local_planning**

Parameters:
* ~local_planning/frame_id (default: ""):
* ~local_planning/frame_lidar (default: ""):
* ~local_planning/save_data (default: false): If it is true, the data will save in a file.
* ~local_planning/url_file_out (default: ""): If save_data is true, define here de path of output data.
* ~lidar_configuration/max_elevation_angle (default: null):
* ~lidar_configuration/min_elevation_angle (default: null):
* ~lidar_configuration/max_azimuth_angle (default: null):
* ~lidar_configuration/min_azimuth_angle (default: null):
* ~lidar_configuration/grid_azimuth_angular_resolution (default: null):
* ~lidar_configuration/grid_elevation_angular_resolution (default: null):
* ~filter_configuration/max_range (default: null):
* ~filter_configuration/min_range (default: null):
* ~filter_configuration/a (default: null):
* ~filter_configuration/b (default: null):
* ~filter_configuration/c (default: null):
* ~filter_configuration/variance (default: null):
* ~filter_configuration/radious (default: null):
* ~filter_configuration/var_factor (default: null):
* ~filter_configuration/ground_in_sim (default: null):
* ~filter_configuration/is_simulation (default: false):
* ~filter_configuration/is_reconfig (default: false):
* ~ackermann_control/max_angle (default: null):
* ~ackermann_control/delta_angle (default: null):
* ~ackermann_control/v_length (default: null):
* ~ackermann_control/delta_arc (default: null):
* ~ackermann_control/max_arc (default: null):
* ~ackermann_control/v_min (default: null):
* ~ackermann_control/v_max (default: null):
* ~ackermann_control/kp (default: null):
* ~ackermann_control/margin (default: null):

**get_pose_from_tf**
This package contains a node that, as input, reads the /tf messages. This node calculates the transformation between two differents frames to obtain a odometry message. The node output is published in the topic /odometry_filtered of type nav_msgs::Odometry.

Parameters:
* ~frame_id_tf (default: ""): Parent frame name to transform.
* ~child_id_tf (default: ""): Child frame name to transform.

**path_planning_demo (DEPRECATED!!)**
This package contains a node that, as input, reads the topics /request_goal of type std_msgs::Bool, /re_locate of type std_msgs::Bool, and /amcl_pose of type geometry_msgs::PoseWithCovariance. The node output is published in the topics /visualization of type visualization_msgs::MarkerArray, and /move_base_simple/goal of type geometry_msgs::PoseStamped.

Parameters:
* ~mode_path (default: 0): 1 -> close loop, 2 -> random trajectories, 3 -> global goal.
* ~frame_id_markers (default: ""): Frame where the markers representing the trajectory will be published..
* ~path_file_links (default: ""): Path to graph links generated with matlab scripts privided in this package.
* ~path_file_nodes (default: ""): Path to graph nodes generated with matlab scripts privided in this package.
* ~path_file_goals (default: ""): Path to graph goals generated with matlab scripts privided in this package.
