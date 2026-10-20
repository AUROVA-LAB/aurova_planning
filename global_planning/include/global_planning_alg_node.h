// Copyright (C) 202019-2026 Grupo de Automática, Robótica y Visión Artificial (AUROVA), Universidad de Alicante, Spain.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _global_planning_alg_node_h_
#define _global_planning_alg_node_h_


#include <planning/graph.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>
#include <fstream>
#include <sstream>

#define PI 3.141592
#define MODE_PATH 1
#define MODE_LOOP 2
#define MODE_BYPASS 3
#define MODE_INIT_LOOP 4

// [publisher subscriber headers]

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class GlobalPlanningAlgNode : public rclcpp::Node
{
private:

  bool init_loop_;
  int index_loop;
  bool flag_pose_;
  bool flag_goal_;
  bool save_data_;
  std::string url_file_out_;
  int operation_mode_;
  int vectors_size_;
  float stop_code_;
  double rad_reached_;
  int index_path_;
  struct Pose pose_;
  struct Pose global_goal_;
  struct Pose slocal_goal_;
  struct Pose utm_to_frame_;
  std::vector <struct StNodes> st_nodes_;
  std::vector <struct Pose> st_path_;
  visualization_msgs::msg::MarkerArray marker_array_;
  std::string frame_id_;
  geometry_msgs::msg::PoseWithCovarianceStamped local_goal_;
  geometry_msgs::msg::TransformStamped tf_to_utm_;
  geometry_msgs::msg::PoseStamped global_goal_bypass_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  Graph *graph_;
  std::vector<int> closed_loop_;

  // [publisher attributes]
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr local_goal_pub_;

  // [subscriber attributes]
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber_;

  /**
   * \brief callback for read pose messages
   * This message can be read from different localization sources by remapping in the
   * execution of the node.
   */
  void cb_getPoseMsg(const nav_msgs::msg::Odometry::SharedPtr  pose_msg);

  /**
   * \brief callback for read pose messages to use as a goal
   * This message can be read from different goal sources by remapping in the
   * execution of the node.
   */
  void cb_getGoalMsg(const geometry_msgs::msg::PoseStamped::SharedPtr  goal_msg);

  // [service attributes]

  // [client attributes]

  // [action server attributes]

  // [action client attributes]

public:
  /**
   * \brief Constructor
   *
   * This constructor initializes specific class attributes and all ROS
   * communications variables to enable message exchange.
   */
  GlobalPlanningAlgNode(void);

  /**
   * \brief Destructor
   *
   * This destructor frees all necessary dynamic memory allocated within this
   * this class.
   */
  ~GlobalPlanningAlgNode(void);

  /**
   * \brief main node thread
   *
   * This is the main thread node function. Code written here will be executed
   * in every node loop while the algorithm is on running state. Loop frequency
   * can be tuned by modifying loop_rate attribute.
   *
   * Here data related to the process loop or to ROS topics (mainly data structs
   * related to the MSG and SRV files) must be updated. ROS publisher objects
   * must publish their data in this process. ROS client servers may also
   * request data to the corresponding server topics.
   */
  void mainNodeThread(void);

  std::shared_ptr<rclcpp::Rate> loop_rate;
  double rate;

protected:
  /**
   * \brief dynamic reconfigure server callback
   *
   * This method is called whenever a new configuration is received through
   * the dynamic reconfigure. The derivated generic algorithm class must
   * implement it.
   *
   * \param config an object with new configuration from all algorithm
   *               parameters defined in the config file.
   * \param level  integer referring the level in which the configuration
   *               has been changed.
   */
	rcl_interfaces::msg::SetParametersResult node_config_update(const std::vector<rclcpp::Parameter> & parameters);
  
  /**
   * \brief Parse the information in alg structure nodes to visualization marker.
   *
   * @param marker is structure for visualization.
   */
  int parseNodesToRosMarker(visualization_msgs::msg::MarkerArray& marker_array);
  
  /**
   * \brief Parse the information in alg structure nodes to visualization marker.
   *
   * @param marker is structure for visualization.
   */
  int parsePathToRosMarker(visualization_msgs::msg::MarkerArray& marker_array);
  
  /**
   * \brief Parse the information in alg structure links to visualization marker.
   *
   * @param marker is structure for visualization.
   */
  int parseLinksToRosMarker(visualization_msgs::msg::MarkerArray& marker_array);
  
  /**
   * \brief Get transform from /utm to /current_frame.
   */
  void fromUtmTransform(void);
  
  /**
   * \brief Get yaw from a path node in next node direction
   */
  double getYawFromPath(int index_path);

  // [diagnostic functions]

  // [test functions]
};

#endif
