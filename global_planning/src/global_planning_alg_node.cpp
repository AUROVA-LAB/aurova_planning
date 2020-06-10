#include "global_planning_alg_node.h"

GlobalPlanningAlgNode::GlobalPlanningAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<GlobalPlanningAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]

  // get aplication parameters
  std::string url_path, type_dist;
  double var_x, var_y, var_z, var_w, rad_reached;
  int vectors_size = 4;
  this->public_node_handle_.getParam("/global_planning/url_path", url_path);
  this->public_node_handle_.getParam("/global_planning/var_x", var_x);
  this->public_node_handle_.getParam("/global_planning/var_y", var_y);
  this->public_node_handle_.getParam("/global_planning/var_z", var_z);
  this->public_node_handle_.getParam("/global_planning/var_w", var_w);
  this->public_node_handle_.getParam("/global_planning/type_dist", type_dist);
  this->public_node_handle_.getParam("/global_planning/rad_reached", rad_reached);

  // set covariance matrix
  std::vector < std::vector<double> > covariance;
  covariance.resize(vectors_size);
  for (int i = 0; i < vectors_size; i++)
    covariance[i].resize(vectors_size);
  covariance[0][0] = var_x;
  covariance[1][1] = var_y;
  covariance[2][2] = var_z;
  covariance[3][3] = var_w;

  // class constructor for graph
  this->graph_ = new Graph(url_path, covariance, type_dist, rad_reached);

  // inicializations of poses
  this->pose_.coordinates.resize(vectors_size);
  this->pose_.matrix.resize(vectors_size);
  for (int i = 0; i < vectors_size; i++)
    this->pose_.matrix[i].resize(vectors_size);
  this->global_goal_.coordinates.resize(vectors_size);
  this->global_goal_.matrix.resize(vectors_size);
  for (int i = 0; i < vectors_size; i++)
    this->global_goal_.matrix[i].resize(vectors_size);
  this->global_goal_.matrix[0][0] = var_x;
  this->global_goal_.matrix[1][1] = var_y;
  this->global_goal_.matrix[2][2] = var_z;
  this->global_goal_.matrix[3][3] = var_w;

  // [init publishers]
  this->marker_pub_ = this->public_node_handle_.advertise < visualization_msgs::MarkerArray > ("/visualization", 1);
  this->local_goal_pub_ = this->public_node_handle_.advertise < geometry_msgs::PoseWithCovarianceStamped
      > ("/semilocal_goal", 1);

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
  static bool first_exec = true;
  if (first_exec)
  {
    first_exec = false;
  }
  this->slocal_goal_ = this->graph_->getNextPose(this->pose_, this->global_goal_);

  tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, this->slocal_goal_.coordinates.at(3));
  this->local_goal_.pose.pose.position.x = this->slocal_goal_.coordinates.at(0);
  this->local_goal_.pose.pose.position.y = this->slocal_goal_.coordinates.at(1);
  this->local_goal_.pose.pose.orientation.x = quaternion[0];
  this->local_goal_.pose.pose.orientation.y = quaternion[1];
  this->local_goal_.pose.pose.orientation.z = quaternion[2];
  this->local_goal_.pose.pose.orientation.w = quaternion[3];
  this->local_goal_.pose.covariance[0] = this->slocal_goal_.matrix[0][0];
  this->local_goal_.pose.covariance[7] = this->slocal_goal_.matrix[1][1];
  this->local_goal_.pose.covariance[14] = this->slocal_goal_.matrix[2][2];
  this->local_goal_.pose.covariance[35] = this->slocal_goal_.matrix[3][3];

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->local_goal_pub_.publish(this->local_goal_);
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

  this->pose_.coordinates.at(0) = pose_msg->pose.pose.position.x;
  this->pose_.coordinates.at(1) = pose_msg->pose.pose.position.y;
  this->pose_.coordinates.at(2) = pose_msg->pose.pose.position.z;
  this->pose_.coordinates.at(3) = yaw;
  this->pose_.matrix[0][0] = pose_msg->pose.covariance[0];
  this->pose_.matrix[1][1] = pose_msg->pose.covariance[7];
  this->pose_.matrix[2][2] = pose_msg->pose.covariance[14];
  this->pose_.matrix[3][3] = pose_msg->pose.covariance[35];

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

  this->global_goal_.coordinates.at(0) = goal_msg->pose.position.x;
  this->global_goal_.coordinates.at(1) = goal_msg->pose.position.y;
  this->global_goal_.coordinates.at(2) = goal_msg->pose.position.z;
  this->global_goal_.coordinates.at(3) = yaw;

  this->local_goal_.header.frame_id = goal_msg->header.frame_id;

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
