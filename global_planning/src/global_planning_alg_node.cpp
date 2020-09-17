#include "global_planning_alg_node.h"

GlobalPlanningAlgNode::GlobalPlanningAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<GlobalPlanningAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]

  //////////////////////////////////////////////////
  // get aplication parameters
  std::string url_path, type_dist;
  double var_x, var_y, var_z, var_w, rad_reached;
  int vectors_size = 4;
  this->public_node_handle_.getParam("/global_planning/url_path", url_path);
  this->public_node_handle_.getParam("/global_planning/frame_id", this->frame_id_);
  this->public_node_handle_.getParam("/global_planning/var_x", var_x);
  this->public_node_handle_.getParam("/global_planning/var_y", var_y);
  this->public_node_handle_.getParam("/global_planning/var_z", var_z);
  this->public_node_handle_.getParam("/global_planning/var_w", var_w);
  this->public_node_handle_.getParam("/global_planning/type_dist", type_dist);
  this->public_node_handle_.getParam("/global_planning/rad_reached", rad_reached);

  //////////////////////////////////////////////////
  // set covariance matrix
  std::vector < std::vector<double> > covariance;
  covariance.resize(vectors_size);
  for (int i = 0; i < vectors_size; i++)
    covariance[i].resize(vectors_size);
  covariance[0][0] = var_x;
  covariance[1][1] = var_y;
  covariance[2][2] = var_z;
  covariance[3][3] = var_w;

  //////////////////////////////////////////////////
  // class constructor for graph
  this->graph_ = new Graph(url_path, covariance, type_dist, rad_reached);

  //////////////////////////////////////////////////
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
  
  //////////////////////////////////////////////////
  // parse graph to a struct array
  this->st_nodes_ = this->graph_->getStructGraph();
  this->parseNodesToRosMarker(this->marker_array_);
  
  
  // [init publishers]
  this->marker_pub_ = this->public_node_handle_.advertise < visualization_msgs::MarkerArray > ("/visualization", 1);
  this->local_goal_pub_ = this->public_node_handle_.advertise < geometry_msgs::PoseWithCovarianceStamped
      > ("/semilocal_goal", 1);

  // [init subscribers]
  this->odom_subscriber_ = this->public_node_handle_.subscribe("/odom", 1, &GlobalPlanningAlgNode::cb_getOdomMsg, this);
  this->pose_subscriber_ = this->public_node_handle_.subscribe("/pose_sim", 1, &GlobalPlanningAlgNode::cb_getPoseMsg, this);
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
  
  /*this->slocal_goal_ = this->graph_->getNextPose(this->pose_, this->global_goal_);

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
  this->local_goal_.pose.covariance[35] = this->slocal_goal_.matrix[3][3];*/

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->local_goal_pub_.publish(this->local_goal_);
  this->marker_pub_.publish(this->marker_array_);
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
void GlobalPlanningAlgNode::cb_getOdomMsg(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  this->alg_.lock();

  double roll, pitch, yaw;
  tf::Quaternion q_pose(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
                        odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
  tf::Matrix3x3 m_pose(q_pose);
  m_pose.getRPY(roll, pitch, yaw);

  this->pose_.coordinates.at(0) = odom_msg->pose.pose.position.x;
  this->pose_.coordinates.at(1) = odom_msg->pose.pose.position.y;
  this->pose_.coordinates.at(2) = odom_msg->pose.pose.position.z;
  this->pose_.coordinates.at(3) = yaw;
  this->pose_.matrix[0][0] = odom_msg->pose.covariance[0];
  this->pose_.matrix[1][1] = odom_msg->pose.covariance[7];
  this->pose_.matrix[2][2] = odom_msg->pose.covariance[14];
  this->pose_.matrix[3][3] = odom_msg->pose.covariance[35];

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

int GlobalPlanningAlgNode::parseNodesToRosMarker(visualization_msgs::MarkerArray& marker_array)
{
  int i;

  visualization_msgs::Marker marker;

  marker.header.frame_id = this->frame_id_;
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "nodes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::SPHERE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  int size_n = this->st_nodes_.size();
  double x_ref = this->st_nodes_[0].coordinates[0];
  double y_ref = this->st_nodes_[0].coordinates[1];
  for (i = 0; i < size_n; i++)
  {
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = this->st_nodes_[i].coordinates[0] - x_ref;
    marker.pose.position.y = this->st_nodes_[i].coordinates[1] - y_ref;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.id = this->st_nodes_[i].id;
    marker_array.markers.push_back(marker);
  }

  return 0;
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < GlobalPlanningAlgNode > (argc, argv, "global_planning_alg_node");
}
