/**
 * \file topologic_planning.h
 *
 *  Created on: 08 Oct. 2018
 *      Author: m.a.munoz
 */

#ifndef _topologic_planning_h_
#define _topologic_planning_h_

#include <fstream>

#define DIRECT_SENSE 1
#define INVERSE_SENSE -1

struct Links
{
  int id_link[2]; // id_link[0]: node ini. id_link[1] node end.
  int num_links;
  int num_points;
  int *point_id;
  float *points_x;
  float *points_y;
};

struct Nodes
{
  int id_node;
  int num_nodes;
  int num_neighbors;
  int *id_neighbors;
  int *index_links;
  int *sense_links;
  float pose_x;
  float pose_y;
};

struct Goals
{
  int id_goal;
  int num_goals;
  int index_link;
  int point_id;
  float pose_x;
  float pose_y;
};

struct Pose
{
  int last_node;
  int index_link;
  int point_id;
  float pose_x;
  float pose_y;
};

struct Path
{
  int num_points;
  int *point_id;
  float *points_x;
  float *points_y;
};

/**
 * \brief Class for path planning
 *
 * This class contains the necessary methods to generate a route using a topological map,
 * which represents possible trajectories, and a target point.
 *
 */
class TopologicPlanning;
typedef TopologicPlanning* TopologicPlanningPtr;

class TopologicPlanning
{
private:

public:

  struct Links *st_links_;
  struct Nodes *st_nodes_;
  struct Goals *st_goals_;
  struct Pose st_pose_;
  struct Path st_path_;

  /**
   * \brief constructor
   *
   * initialization of necessary parameters.
   */
  TopologicPlanning(void);

  /**
   * \brief Destructor
   *
   * This destructor is called when the object is about to be destroyed.
   */
  ~TopologicPlanning(void);

  /**
   * \brief generate path
   *
   * This method generate path using the natural sorting of the links starting in the box point.
   * USED ONLY FOR CIRCULAR PATH!!!
   */
  int generateSecPathFromBox(void);

  /**
   * \brief load links file
   *
   * This method reads the information of links of trajectories structured in a topological way,
   * and it is stored in the class structures.
   *
   * @param filePath is the path of file
   */
  int loadLinksFromFile(std::string filePath);

  /**
   * \brief load nodes file
   *
   * This method reads the information of nodes of trajectories structured in a topological way,
   * and it is stored in the class structures. This method needs the link structure
   * initialized, that is, it needs the "loadLinksFromFile()" method to be executed beforehand.
   *
   * @param filePath is the path of file
   */
  int loadNodesFromFile(std::string filePath);

  /**
   * \brief load global goals file
   *
   * This method reads the information of global goals of trajectories,
   * and it is stored in the class structures. This method needs the link structure
   * initialized, that is, it needs the "loadLinksFromFile()" method to be executed beforehand.
   *
   * @param filePath is the path of file
   */
  int loadGoalsFromFile(std::string filePath);
};

#endif
