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

struct Goals
{
  int id_goal;
  int num_goals;
  int index_link;
  float pose_x;
  float pose_y;
};

struct Pose
{
  int index_link;
  int last_node;
  float pose_x;
  float pose_y;
};

struct Links
{
  int id_link[2]; // id_link[0]: node ini. id_link[1] node end.
  int num_links;
  int num_points;
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

  struct Goals *st_goals_;
  struct Pose *st_pose_;
  struct Links *st_links_;
  struct Nodes *st_nodes_;

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
   * \brief load links file
   *
   * This method reads the information of links of trajectories structured in a topological way,
   * and it is stored in the class structures.
   *
   * @param filePath is the path of file
   */
  int loadLinksFromFile(char *filePath);

  /**
   * \brief load nodes file
   *
   * This method reads the information of nodes of trajectories structured in a topological way,
   * and it is stored in the class structures.
   *
   * @param filePath is the path of file
   */
  int loadNodesFromFile(char *filePath);

  /**
   * \brief load goals file
   *
   * This method reads the information of goals of trajectories,
   * and it is stored in the class structures.
   *
   * @param filePath is the path of file
   */
  int loadGoalsFromFile(char *filePath);
};

#endif
