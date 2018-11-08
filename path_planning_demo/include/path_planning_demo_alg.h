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

#ifndef _path_planning_demo_alg_h_
#define _path_planning_demo_alg_h_

#define SECUENTIAL_FROM_BOX 1
#define RANDOM_FROM_BOX 2
#define GLOBAL_GOAL 3
#define NO_ACTION 0
#define NEW_LOCAL_GOAL 1
#define END_PATH 2

#include <path_planning_demo/PathPlanningDemoConfig.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
#include "topologic_planning.h"

//include path_planning_demo_alg main library

/**
 * \brief IRI ROS Specific Driver Class
 *
 *
 */
class PathPlanningDemoAlgorithm
{
  protected:
   /**
    * \brief define config type
    *
    * Define a Config type with the PathPlanningDemoConfig. All driver implementations
    * will then use the same variable type Config.
    */
    pthread_mutex_t access_;    

    // private attributes and methods

  public:

    TopologicPlanningPtr planning_;
    std::string frame_id_markers_;

   /**
    * \brief define config type
    *
    * Define a Config type with the PathPlanningDemoConfig. All driver implementations
    * will then use the same variable type Config.
    */
    typedef path_planning_demo::PathPlanningDemoConfig Config;

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config_;

   /**
    * \brief constructor
    *
    * In this constructor parameters related to the specific driver can be
    * initalized. Those parameters can be also set in the openDriver() function.
    * Attributes from the main node driver class IriBaseDriver such as loop_rate,
    * may be also overload here.
    */
    PathPlanningDemoAlgorithm(void);

   /**
    * \brief Lock Algorithm
    *
    * Locks access to the Algorithm class
    */
    void lock(void) { pthread_mutex_lock(&this->access_); };

   /**
    * \brief Unlock Algorithm
    *
    * Unlocks access to the Algorithm class
    */
    void unlock(void) { pthread_mutex_unlock(&this->access_); };

   /**
    * \brief Tries Access to Algorithm
    *
    * Tries access to Algorithm
    * 
    * \return true if the lock was adquired, false otherwise
    */
    bool try_enter(void) 
    { 
      if(pthread_mutex_trylock(&this->access_)==0)
        return true;
      else
        return false;
    };

   /**
    * \brief config update
    *
    * In this function the driver parameters must be updated with the input
    * config variable. Then the new configuration state will be stored in the 
    * Config attribute.
    *
    * \param new_cfg the new driver configuration state
    *
    * \param level level in which the update is taken place
    */
    void config_update(Config& config, uint32_t level=0);

    // here define all path_planning_demo_alg interface methods to retrieve and set
    // the driver parameters

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~PathPlanningDemoAlgorithm(void);

    /**
     * \brief Load the information of the graph to local structures.
     *
     * @param pathLinks is the path to link files.
     * @param pathNodes is the path to nodes files.
     * @param pathGoals is the path to goals files.
     */
    int readGraphFromFile(std::string pathLinks, std::string pathNodes, std::string pathGoals);

    /**
     * \brief Manage the cration of the path, and the sends of local goals.
     *
     * In this method the creation of the path is managed depend on the mode. Also, if
     * we detect up flank in flag_request_goal, the following local goal is sent parsed in local_goal.
     * This method return integer status: NO_ACTION = 0, NEW_LOCAL_GOAL = 1, or END_PATH = 2.
     *
     * @param local_goal (output) is ROS structure for goal representation.
     * @param flag_request_goal (input) flag for goal request.
     * @param mode (imput) this is the mode of creating path.
     */
    int managePath(geometry_msgs::PoseStamped& local_goal, bool flag_request_goal, int mode);
};

#endif
