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

#ifndef _path_planning_demo_alg_node_h_
#define _path_planning_demo_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "path_planning_demo_alg.h"

// [publisher subscriber headers]

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class PathPlanningDemoAlgNode : public algorithm_base::IriBaseAlgorithm<PathPlanningDemoAlgorithm>
{
private:

  int mode_path_;
  std::string path_file_links_;
  std::string path_file_nodes_;
  std::string path_file_goals_;
  uint32_t shape_;
  visualization_msgs::Marker marker_;
  visualization_msgs::MarkerArray marker_array_;
  geometry_msgs::PoseStamped local_goal_;
  std_msgs::Bool flag_request_goal_;

  // [publisher attributes]
  ros::Publisher marker_pub_;
  ros::Publisher local_goal_pub_;

  // [subscriber attributes]
  ros::Subscriber request_goal_sub_;
  ros::Subscriber pose_subscriber_;
  ros::Subscriber reloc_sub_;

  /**
   * \brief callback for read flag for listen request for new goal
   */
  void cb_getRequestGoalMsg(const std_msgs::Bool::ConstPtr& flag_msg);

  /**
   * \brief callback for read flag for relocate index of path
   */
  void cb_getRelocateMsg(const std_msgs::Bool::ConstPtr& flag_msg);

  /**
   * \brief callback for read pose messages
   * This message can be read from different localization sources by remapping in the
   * execution of the node.
   */
  void cb_getPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);

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
  PathPlanningDemoAlgNode(void);

  /**
   * \brief Destructor
   *
   * This destructor frees all necessary dynamic memory allocated within this
   * this class.
   */
  ~PathPlanningDemoAlgNode(void);

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
   * \brief Parse the information in alg structure links to visualization marker.
   *
   * @param marker is structure for visualization.
   */
  int parseLinksToRosMarker(visualization_msgs::MarkerArray& marker_array);

  /**
   * \brief Parse the information in alg structure nodes to visualization marker.
   *
   * @param marker is structure for visualization.
   */
  int parseNodesToRosMarker(visualization_msgs::MarkerArray& marker_array);

  /**
   * \brief Parse the information in alg structure goals to visualization marker.
   *
   * @param marker is structure for visualization.
   */
  int parseGoalsToRosMarker(visualization_msgs::MarkerArray& marker_array);

  // [diagnostic functions]

  // [test functions]
};

#endif
