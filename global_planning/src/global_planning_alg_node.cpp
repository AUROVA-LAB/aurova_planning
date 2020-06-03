#include "global_planning_alg_node.h"

GlobalPlanningAlgNode::GlobalPlanningAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<GlobalPlanningAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]

  // [init publishers]

  // [init subscribers]
  this->pose_subscriber_ = this->public_node_handle_.subscribe("/pose_sim", 1, &GlobalPlanningAlgNode::cb_getPoseMsg,
                                                               this);
  this->goal_subscriber_ = this->public_node_handle_.subscribe("/move_base_simple/goal", 1,
                                                               &GlobalPlanningAlgNode::cb_getGoalMsg, this);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

GlobalPlanningAlgNode::~GlobalPlanningAlgNode(void)
{
  // [free dynamic memory]
}

void GlobalPlanningAlgNode::mainNodeThread(void)
{
  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */
void GlobalPlanningAlgNode::cb_getPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  this->alg_.lock();

  double roll, pitch, yaw;
  tf::Quaternion q_pose(pose_msg->pose.pose.orientation.x, pose_msg->pose.pose.orientation.y,
                        pose_msg->pose.pose.orientation.z, pose_msg->pose.pose.orientation.w);
  tf::Matrix3x3 m_pose(q_pose);
  m_pose.getRPY(roll, pitch, yaw);

  /*
  this->pose_.coordinates.at(0) = pose_msg->pose.pose.position.x;
  this->pose_.coordinates.at(1) = pose_msg->pose.pose.position.y;
  this->pose_.coordinates.at(2) = pose_msg->pose.pose.position.z;
  this->pose_.coordinates.at(3) = yaw;
  this->pose_.matrix[0][0] = 1.0; //pose_msg->pose.covariance[0];
  this->pose_.matrix[1][1] = 1.0; //pose_msg->pose.covariance[7];
  this->pose_.matrix[2][2] = 1.0; //pose_msg->pose.covariance[14];
  this->pose_.matrix[3][3] = 0.001; //pose_msg->pose.covariance[35];
  */
  //ROS_INFO("pose -> x: %f, y: %f", this->pose_.coordinates.at(1), this->pose_.matrix[1].at(1));
  this->alg_.unlock();
}
void GlobalPlanningAlgNode::cb_getGoalMsg(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
  this->alg_.lock();

  double roll, pitch, yaw;
  tf::Quaternion q_pose(goal_msg->pose.orientation.x, goal_msg->pose.orientation.y, goal_msg->pose.orientation.z,
                        goal_msg->pose.orientation.w);
  tf::Matrix3x3 m_pose(q_pose);
  m_pose.getRPY(roll, pitch, yaw);
  /*
  this->goal_.coordinates.at(0) = goal_msg->pose.position.x;
  this->goal_.coordinates.at(1) = goal_msg->pose.position.y;
  this->goal_.coordinates.at(2) = goal_msg->pose.position.z;
  this->goal_.coordinates.at(3) = yaw;
  this->goal_.matrix[0][0] = 1.0;
  this->goal_.matrix[1][1] = 1.0;
  this->goal_.matrix[2][2] = 1.0;
  this->goal_.matrix[3][3] = 0.001;
  */
  ROS_INFO("goal -> x: %f, y: %f", goal_msg->pose.position.x, goal_msg->pose.position.y);
  this->alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void GlobalPlanningAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void GlobalPlanningAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < GlobalPlanningAlgNode > (argc, argv, "global_planning_alg_node");
}
