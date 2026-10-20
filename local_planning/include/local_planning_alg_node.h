// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
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

#ifndef _local_planning_alg_node_h_
#define _local_planning_alg_node_h_

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "std_msgs/msg/float32.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>
#include <fstream>
#include <planning/local_planning.h>


// [publisher subscriber headers]

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class LocalPlanningAlgNode : public rclcpp::Node
{
private:

  LocalPlanning *local_planning_;
  local_planning_lib::SensorConfiguration lidar_config_;
  local_planning_lib::FilteringConfiguration filter_config_;
  local_planning_lib::AckermannControl ackermann_control_;
  local_planning_lib::Pose2D base_in_lidarf_;
  ackermann_msgs::msg::AckermannDrive ackermann_state_rad_;
  ackermann_msgs::msg::AckermannDrive ackermann_state_deg_;
  pcl::PointXYZ goal_lidar_;
  pcl::PointXYZ pos_frame_;
  std::string frame_id_;
  std::string frame_lidar_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::string url_file_out_;
  bool save_data_;
  bool goal_received_;
  bool ctrl_received_;
  bool is_reconfig_;
  float stop_code_;

  // [publisher attributes]
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr limits_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_goal_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr collision_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr collision_actions_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr free_actions_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_publisher_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_rad_publisher_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_deg_publisher_;

  // [subscriber attributes]
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr goal_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber_;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sync_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> obstacle_sync_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> ground_obstacle_sync_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> ground_limits_sync_subscriber_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, 
                                                sensor_msgs::msg::PointCloud2,sensor_msgs::msg::PointCloud2> MySyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;

  /**
   * \brief Callback for read lidar messages.
   */
  void cb_lidarInfo(const sensor_msgs::msg::PointCloud2::SharedPtr scan);
  void cb_lidarInfo2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&  scan,const sensor_msgs::msg::PointCloud2::ConstSharedPtr&  obstacles, 
                    const sensor_msgs::msg::PointCloud2::ConstSharedPtr&  ground_obstacles, const sensor_msgs::msg::PointCloud2::ConstSharedPtr&  ground_limits);

  /**
   * \brief callback for read pose messages to use as a goal
   * This message can be read from different goal sources by remapping in the
   * execution of the node.
   */
  void cb_getGoalMsg(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr goal_msg);

  /**
   * \brief callback for read pose messages
   * This message can be read from different localization sources by remapping in the
   * execution of the node.
   */
  void cb_getPoseMsg(const nav_msgs::msg::Odometry::SharedPtr  pose_msg);

  void publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, std::string frame);

  // [service attributes]

  // [client attributes]

  // [action server attributes]

  // [action client attributes]

  /**
   * \brief config variable
   *
   * This variable has all the driver parameters defined in the cfg config file.
   * Is updated everytime function config_update() is called.
   */

public:
  /**
   * \brief Constructor
   *
   * This constructor initializes specific class attributes and all ROS
   * communications variables to enable message exchange.
   */
  LocalPlanningAlgNode(void);

  /**
   * \brief Destructor
   *
   * This destructor frees all necessary dynamic memory allocated within this
   * this class.
   */
  ~LocalPlanningAlgNode(void);

  void mainNodeThread(void);
  std::shared_ptr<rclcpp::Rate> loop_rate;
  double rate;

protected:
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

  // [diagnostic functions]

  // [test functions]
};

#endif