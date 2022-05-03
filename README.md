### Example of usage:

You can run an example following the instructions in: [application_navigation](https://github.com/AUROVA-LAB/application_navigation) and [application_localization](https://github.com/AUROVA-LAB/application_localization).

# aurova_planning
This is a metapackage that contains different packages that perform planning algorithims. Compiling this metapackage into ROS will compile all the packages at once. This metapackage is grouped as a project for eclipse C++. Each package contains a "packagename_doxygen_config" configuration file for generate doxygen documentation. The packages contained in this metapackage are:

**global_planning**
This package contains a node that, as input, reads /pose_plot topic of type geometry_msgs::PoseWithCovarianceStamped, and /move_base_simple/goal topic of type geometry_msgs::PoseStamped. With this information, this package calculates the best path in a OSM graph, an retuns the neares node of the calculate path as /semilocal_goal topic of type geometry_msgs::PoseWithCovarianceStamped.

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
* ~global_planning/url_file_out (default: ""): If save_data is true, define here the path of output data.

**local_planning**
This package contains a node that, as input, reads /velodyne_points of type sensor_msgs::PointCloud2, /semilocal_goal of type geometry_msgs::PoseWithCovarianceStamped, and /pose_plot of type geometry_msgs::PoseWithCovarianceStamped. With these information, the package infers the ackerman control actions in a topic /desired_ackermann_state of type ackermann_msgs::AckermannDriveStamped for real case, and /ackermann_cmd of type ackermann_msgs::AckermannDrive for gazebo simulation case.

Parameters:
* ~local_planning/frame_id (default: ""): Frame for local planning (usually "map").
* ~local_planning/frame_lidar (default: ""): Frame for LiDAR sensor used (usually "velodyne").
* ~local_planning/save_data (default: false): If it is true, the data will save in a file.
* ~local_planning/url_file_out (default: ""): If save_data is true, define here the path of output data.
* ~lidar_configuration/max_elevation_angle (default: null): Max elevation for lidar beams.
* ~lidar_configuration/min_elevation_angle (default: null): Min elevation for lidar beams.
* ~lidar_configuration/max_azimuth_angle (default: null): Max azimuth for lidar beams.
* ~lidar_configuration/min_azimuth_angle (default: null): Min azimuth for lidar beams.
* ~lidar_configuration/grid_azimuth_angular_resolution (default: null): Azimuth resolution for lidar beams.
* ~lidar_configuration/grid_elevation_angular_resolution (default: null): Elevation resolution for lidar beams.
* ~filter_configuration/max_range (default: null): Max range for lidar beams.
* ~filter_configuration/min_range (default: null): Min range for lidar beams
* ~filter_configuration/a (default: null): Plane parameters (previously adjusted in optimization software).
* ~filter_configuration/b (default: null): Plane parameters (previously adjusted in optimization software).
* ~filter_configuration/c (default: null): Plane parameters (previously adjusted in optimization software).
* ~filter_configuration/variance (default: null): Variance for consider points part of the plane.
* ~filter_configuration/radious (default: null): To filter points out of this radious.
* ~filter_configuration/var_factor (default: null): To do variance proportional to the distance respect to the center of the sensor.
* ~filter_configuration/ground_in_sim (default: null): To filter ground in simulation case.
* ~filter_configuration/is_simulation (default: false): It shoul be true for gazebo simulation case, and false for real or .bag file case.
* ~filter_configuration/is_reconfig (default: false): If it is true, the rqt could be use to reconfigure parameters.
* ~ackermann_control/max_angle (default: null): Max steering angle of used vehicle.
* ~ackermann_control/delta_angle (default: null): Resolution of steering angle for control actions.
* ~ackermann_control/v_length (default: null): Length between axles of used vehicle.
* ~ackermann_control/delta_arc (default: null): Steps for estimated trajectory arcs.
* ~ackermann_control/max_arc (default: null): Max value for estimated trajectory arcs.
* ~ackermann_control/v_min (default: null): Min velocity considered for used vehicle.
* ~ackermann_control/v_max (default: null): Max velocity considered for used vehicle.
* ~ackermann_control/kp (default: null): Proportional change value for velocity control. If 1.0, proportional behavior unactivated.
* ~ackermann_control/margin (default: null): Security margin computed from base_link frame.

**path_planning_demo (DEPRECATED!!)**
This package contains a node that, as input, reads the topics /request_goal of type std_msgs::Bool, /re_locate of type std_msgs::Bool, and /amcl_pose of type geometry_msgs::PoseWithCovariance. The node output is published in the topics /visualization of type visualization_msgs::MarkerArray, and /move_base_simple/goal of type geometry_msgs::PoseStamped.

Parameters:
* ~mode_path (default: 0): 1 -> close loop, 2 -> random trajectories, 3 -> global goal.
* ~frame_id_markers (default: ""): Frame where the markers representing the trajectory will be published..
* ~path_file_links (default: ""): Path to graph links generated with matlab scripts privided in this package.
* ~path_file_nodes (default: ""): Path to graph nodes generated with matlab scripts privided in this package.
* ~path_file_goals (default: ""): Path to graph goals generated with matlab scripts privided in this package.
