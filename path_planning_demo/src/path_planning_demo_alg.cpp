#include "path_planning_demo_alg.h"

PathPlanningDemoAlgorithm::PathPlanningDemoAlgorithm(void)
{
  this->planning_ = new TopologicPlanning();

  pthread_mutex_init(&this->access_, NULL);
}

PathPlanningDemoAlgorithm::~PathPlanningDemoAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void PathPlanningDemoAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_ = config;

  this->unlock();
}

//////////////////////////////////////////////////////////////
// PathPlanningDemoAlgorithm Public API
//////////////////////////////////////////////////////////////

int PathPlanningDemoAlgorithm::managePath(geometry_msgs::PoseStamped& local_goal, bool flag_request_goal, int mode)
{
  static int goal_index;
  static bool first_exec = true;
  static bool flag_request_goal_prev = false;
  int status;
  tf::Quaternion quaternion;

  //////////////////////////////////////////////////////////
  // Manage the generation of the path.
  if (first_exec && mode == SECUENTIAL_FROM_BOX)
  {
    this->planning_->generateSecPathFromBox();
    goal_index = 0;
  }
  else if (first_exec && mode == RANDOM_FROM_BOX)
  {
    //this->planning_->generateRandomPathFromBox();
    goal_index = 0;
  }
  else if (/*new_path &&*/mode == GLOBAL_GOAL)
  {
    //this->planning_->generatePathToGlobalGoal();
    goal_index = 0;
  }
  //////////////////////////////////////////////////////////


  //////////////////////////////////////////////////////////
  // Find lost index in path

  //////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////
  // Management of the sending of goals.
  if (!flag_request_goal_prev && flag_request_goal)
  {
    if (goal_index < this->planning_->st_path_.num_points)
    {
      float orientation_yaw = 0.0;
      if (goal_index > 0)
      {
        float x1 = this->planning_->st_path_.points_x[goal_index - 1];
        float x2 = this->planning_->st_path_.points_x[goal_index];
        float y1 = this->planning_->st_path_.points_y[goal_index - 1];
        float y2 = this->planning_->st_path_.points_y[goal_index];
        orientation_yaw = atan2(y2 - y1, x2 - x1);
      }
      else
      {
        float x1 = this->planning_->st_path_.points_x[goal_index];
        float x2 = this->planning_->st_path_.points_x[goal_index + 1];
        float y1 = this->planning_->st_path_.points_y[goal_index];
        float y2 = this->planning_->st_path_.points_y[goal_index + 1];
        orientation_yaw = atan2(y2 - y1, x2 - x1);
      }
      quaternion = tf::createQuaternionFromRPY(0.0, 0.0, orientation_yaw);

      local_goal.header.frame_id = this->frame_id_markers_;
      local_goal.pose.position.x = this->planning_->st_path_.points_x[goal_index];
      local_goal.pose.position.y = this->planning_->st_path_.points_y[goal_index];
      local_goal.pose.orientation.x = quaternion[0];
      local_goal.pose.orientation.y = quaternion[1];
      local_goal.pose.orientation.z = quaternion[2];
      local_goal.pose.orientation.w = quaternion[3];
      goal_index++;
      status = NEW_LOCAL_GOAL;
    }
    else
    {
      // This is for reboot circular path
      if (mode == SECUENTIAL_FROM_BOX)
      {
        goal_index = 2;
      }
      status = END_PATH;
    }
  }
  else
  {
    status = NO_ACTION;
  }
  /////////////////////////////////////////////////////////

  // For next execution.
  flag_request_goal_prev = flag_request_goal;
  first_exec = false;

  return status;
}

int PathPlanningDemoAlgorithm::readGraphFromFile(std::string pathLinks, std::string pathNodes, std::string pathGoals)
{
  this->planning_->loadLinksFromFile(pathLinks);

  this->planning_->loadNodesFromFile(pathNodes);

  this->planning_->loadGoalsFromFile(pathGoals);

  return 0;
}
