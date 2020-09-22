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

#ifndef _global_planning_alg_node_h_
#define _global_planning_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "global_planning_alg.h"

// [publisher subscriber headers]

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class GlobalPlanningAlgNode : public algorithm_base::IriBaseAlgorithm<GlobalPlanningAlgorithm>
{
private:

  bool flag_pose_;
  bool flag_goal_;
  int vectors_size_;
  struct Pose pose_;
  struct Pose global_goal_;
  struct Pose slocal_goal_;
  struct Pose utm_to_frame_;
  std::vector <struct StNodes> st_nodes_;
  std::vector <struct Pose> st_path_;
  visualization_msgs::MarkerArray marker_array_;
  std::string frame_id_;
  geometry_msgs::PoseWithCovarianceStamped local_goal_;
  Graph *graph_;

  // [publisher attributes]
  ros::Publisher marker_pub_;
  ros::Publisher local_goal_pub_;

  // [subscriber attributes]
  ros::Subscriber goal_subscriber_;
  ros::Subscriber pose_subscriber_;
  ros::Subscriber odom_subscriber_;

  /**
   * \brief callback for read pose messages
   * This message can be read from different localization sources by remapping in the
   * execution of the node.
   */
  void cb_getPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);
  
  /**
   * \brief callback for read odometry messages
   * This message can be read from different localization sources by remapping in the
   * execution of the node.
   */
  void cb_getOdomMsg(const nav_msgs::Odometry::ConstPtr& odom_msg);

  /**
   * \brief callback for read pose messages to use as a goal
   * This message can be read from different goal sources by remapping in the
   * execution of the node.
   */
  void cb_getGoalMsg(const geometry_msgs::PoseStamped::ConstPtr& goal_msg);

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
  Config config_;
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
  void mainNodeThread(void);

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
  void node_config_update(Config &config, uint32_t level);

  /**
   * \brief node add diagnostics
   *
   * In this abstract function additional ROS diagnostics applied to the
   * specific algorithms may be added.
   */
  void addNodeDiagnostics(void);
  
  /**
   * \brief Parse the information in alg structure nodes to visualization marker.
   *
   * @param marker is structure for visualization.
   */
  int parseNodesToRosMarker(visualization_msgs::MarkerArray& marker_array);
  
  /**
   * \brief Parse the information in alg structure links to visualization marker.
   *
   * @param marker is structure for visualization.
   */
  int parseLinksToRosMarker(visualization_msgs::MarkerArray& marker_array);
  
  /**
   * \brief Get transform from /utm to /current_frame.
   */
  void fromUtmTransform(void);

  // [diagnostic functions]

  // [test functions]
};

#endif
