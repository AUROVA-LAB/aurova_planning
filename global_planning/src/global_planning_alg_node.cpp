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
  this->vectors_size_ = 4;
  this->flag_pose_ = false;
  this->flag_goal_ = false;
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
  covariance.resize(this->vectors_size_);
  for (int i = 0; i < this->vectors_size_; i++)
    covariance[i].resize(this->vectors_size_);
  covariance[0][0] = var_x;
  covariance[1][1] = var_y;
  covariance[2][2] = var_z;
  covariance[3][3] = var_w;

  //////////////////////////////////////////////////
  // class constructor for graph
  this->graph_ = new Graph(url_path, covariance, type_dist, rad_reached);

  //////////////////////////////////////////////////
  // inicializations of poses
  this->pose_.coordinates.resize(this->vectors_size_);
  this->pose_.matrix.resize(this->vectors_size_);
  for (int i = 0; i < this->vectors_size_; i++)
    this->pose_.matrix[i].resize(this->vectors_size_);
  this->global_goal_.coordinates.resize(this->vectors_size_);
  this->global_goal_.matrix.resize(this->vectors_size_);
  for (int i = 0; i < this->vectors_size_; i++)
    this->global_goal_.matrix[i].resize(this->vectors_size_);
  this->global_goal_.matrix[0][0] = var_x;
  this->global_goal_.matrix[1][1] = var_y;
  this->global_goal_.matrix[2][2] = var_z;
  this->global_goal_.matrix[3][3] = var_w;
  
  //////////////////////////////////////////////////
  // parse graph to a struct array
  this->st_nodes_ = this->graph_->getStructGraph();
  this->fromUtmTransform();
  
  
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
  static bool first_exec = true;
  if (first_exec)
  {
    this->parseNodesToRosMarker(this->marker_array_);
    this->parseLinksToRosMarker(this->marker_array_);
  }
  
  
  if (this->flag_pose_ && this->flag_goal_)
  {
    
    this->slocal_goal_ = this->graph_->getNextPose(this->pose_, this->global_goal_);
    this->st_path_ = this->graph_->getPathPoses(this->pose_, this->global_goal_);
    
    ///////////////////////////////////////////////////////////
    //// DEBUG UTM
    ROS_INFO("pose => x: %f, y: %f, z: %f, yaw: %f", this->pose_.coordinates[0], this->pose_.coordinates[1],
                                                     this->pose_.coordinates[2], this->pose_.coordinates[3]);
    ROS_INFO("goal => x: %f, y: %f, z: %f, yaw: %f", this->global_goal_.coordinates[0], this->global_goal_.coordinates[1],
                                                     this->global_goal_.coordinates[2], this->global_goal_.coordinates[3]);
    ROS_INFO("node => x: %f, y: %f, z: %f, yaw: %f", this->st_nodes_[0].coordinates[0], this->st_nodes_[0].coordinates[1],
                                                     this->st_nodes_[0].coordinates[2], this->st_nodes_[0].coordinates[3]);
    ROS_INFO("localg => x: %f, y: %f, z: %f, yaw: %f", this->slocal_goal_.coordinates[0], this->slocal_goal_.coordinates[1],
                                                       this->slocal_goal_.coordinates[2], this->slocal_goal_.coordinates[3]);
    
    
    ///////////////////////////////////////////////////////////
    
    
    
    ///////////////////////////////////////////////////////////
    //// DEBUG
    int np = this->st_path_.size();
    ROS_INFO("number path: %d", np);
    for(int i = 0; i<np; i++)
    {
      ///////////////////////////////////////////////////////////
      ///// TRANSFORM TO TF FARME
      geometry_msgs::PointStamped node_tf;
      geometry_msgs::PointStamped node_utm;
      node_utm.header.frame_id = "utm";
      node_utm.header.stamp = ros::Time(0); //ros::Time::now();
      node_utm.point.x = this->st_path_[i].coordinates[0];
      node_utm.point.y = this->st_path_[i].coordinates[1];
      node_utm.point.z = this->st_path_[i].coordinates[2];
      try
      {
        this->listener_.transformPoint(this->frame_id_, node_utm, node_tf);

      }
      catch (tf::TransformException& ex)
      {
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        return;
      }
      ROS_INFO("path n %d => x: %f, y: %f, z: %f, yaw: %f", i, this->st_path_[i].coordinates[0], this->st_path_[i].coordinates[1],
                                                               this->st_path_[i].coordinates[2], this->st_path_[i].coordinates[3]);
                                                               //node_tf.point.x, node_tf.point.y, node_tf.point.z);
      ///////////////////////////////////////////////////////////
    }
    ///////////////////////////////////////////////////////////
    
    
    ///////////////////////////////////////////////////////////
    ///// TRANSFORM TO TF FARME
    geometry_msgs::PointStamped node_tf;
    geometry_msgs::PointStamped node_utm;
    node_utm.header.frame_id = "utm";
    node_utm.header.stamp = ros::Time(0); //ros::Time::now();
    node_utm.point.x = this->slocal_goal_ .coordinates[0];
    node_utm.point.y = this->slocal_goal_ .coordinates[1];
    node_utm.point.z = this->slocal_goal_ .coordinates[2];
    try
    {
      this->listener_.transformPoint(this->frame_id_, node_utm, node_tf);

    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
      return;
    }
    double yaw = this->slocal_goal_.coordinates[3];
    yaw = (yaw * PI) / 180.0;
    tf::Quaternion quat_utm = tf::createQuaternionFromRPY(0, 0, yaw);
    geometry_msgs::QuaternionStamped orient_tf;
    geometry_msgs::QuaternionStamped orient_utm;
    orient_utm.header.frame_id = "utm";
    orient_utm.header.stamp = ros::Time(0);
    orient_utm.quaternion.x =  quat_utm[0];
    orient_utm.quaternion.y =  quat_utm[1];
    orient_utm.quaternion.z =  quat_utm[2];
    orient_utm.quaternion.w =  quat_utm[3];
    try
    {
      this->listener_.transformQuaternion(this->frame_id_, orient_utm, orient_tf);

    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
      return;
    }
    ///////////////////////////////////////////////////////////

    this->local_goal_.header.frame_id = this->frame_id_;
    this->local_goal_.pose.pose.position.x = node_tf.point.x;
    this->local_goal_.pose.pose.position.y = node_tf.point.y;
    this->local_goal_.pose.pose.position.z = 0.0;
    this->local_goal_.pose.pose.orientation.x = orient_tf.quaternion.x;
    this->local_goal_.pose.pose.orientation.y = orient_tf.quaternion.y;
    this->local_goal_.pose.pose.orientation.z = orient_tf.quaternion.z;
    this->local_goal_.pose.pose.orientation.w = orient_tf.quaternion.w;
    this->local_goal_.pose.covariance[0] = this->slocal_goal_.matrix[0][0];
    this->local_goal_.pose.covariance[7] = this->slocal_goal_.matrix[1][1];
    this->local_goal_.pose.covariance[14] = this->slocal_goal_.matrix[2][2];
    this->local_goal_.pose.covariance[35] = this->slocal_goal_.matrix[3][3];
    
    this->local_goal_pub_.publish(this->local_goal_);
    
  }

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->tf_to_utm_.header.seq = this->tf_to_utm_.header.seq + 1;
  this->tf_to_utm_.header.stamp = ros::Time::now();
  this->broadcaster_.sendTransform(this->tf_to_utm_);
  this->marker_pub_.publish(this->marker_array_);
}

/*  [subscriber callbacks] */
void GlobalPlanningAlgNode::cb_getPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  this->alg_.lock();

  ///////////////////////////////////////////////////////////
  ///// TRANSFORM TO TF FARME
  geometry_msgs::PointStamped node_tf;
  geometry_msgs::PointStamped node_utm;
  node_tf.header.frame_id = this->frame_id_;
  node_tf.header.stamp = ros::Time(0); //ros::Time::now();
  node_tf.point.x = pose_msg->pose.pose.position.x;
  node_tf.point.y = pose_msg->pose.pose.position.y;
  node_tf.point.z = pose_msg->pose.pose.position.z;
  try
  {
    this->listener_.transformPoint("utm", node_tf, node_utm);

  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }
  
  geometry_msgs::QuaternionStamped orient_tf;
  geometry_msgs::QuaternionStamped orient_utm;
  orient_tf.header.frame_id = this->frame_id_;
  orient_tf.header.stamp = ros::Time(0);
  orient_tf.quaternion.x = pose_msg->pose.pose.orientation.x;
  orient_tf.quaternion.y = pose_msg->pose.pose.orientation.y;
  orient_tf.quaternion.z = pose_msg->pose.pose.orientation.z;
  orient_tf.quaternion.w = pose_msg->pose.pose.orientation.w;
  try
  {
    this->listener_.transformQuaternion("utm", orient_tf, orient_utm);

  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }
  double roll, pitch, yaw;
  tf::Quaternion q_pose(orient_utm.quaternion.x, orient_utm.quaternion.y,
                        orient_utm.quaternion.z, orient_utm.quaternion.w);
  tf::Matrix3x3 m_pose(q_pose);
  m_pose.getRPY(roll, pitch, yaw);
  yaw = (yaw * 180.0) / PI;
  ///////////////////////////////////////////////////////////
  
  // pose in utm frame
  this->pose_.coordinates.at(0) = node_utm.point.x;
  this->pose_.coordinates.at(1) = node_utm.point.y;
  this->pose_.coordinates.at(2) = node_utm.point.z;
  this->pose_.coordinates.at(3) = yaw;
  this->pose_.matrix[0][0] = pose_msg->pose.covariance[0];
  this->pose_.matrix[1][1] = pose_msg->pose.covariance[7];
  this->pose_.matrix[2][2] = pose_msg->pose.covariance[14];
  this->pose_.matrix[3][3] = pose_msg->pose.covariance[35];
  
  this->flag_pose_ = true;

  this->alg_.unlock();
}
void GlobalPlanningAlgNode::cb_getOdomMsg(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  this->alg_.lock();
  
  
  ///////////////////////////////////////////////////////////
  ///// TRANSFORM TO TF FARME
  geometry_msgs::PointStamped node_tf;
  geometry_msgs::PointStamped node_utm;
  node_tf.header.frame_id = this->frame_id_;
  node_tf.header.stamp = ros::Time(0); //ros::Time::now();
  node_tf.point.x = odom_msg->pose.pose.position.x;
  node_tf.point.y = odom_msg->pose.pose.position.y;
  node_tf.point.z = odom_msg->pose.pose.position.z;
  try
  {
    this->listener_.transformPoint("utm", node_tf, node_utm);

  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }
  
  geometry_msgs::QuaternionStamped orient_tf;
  geometry_msgs::QuaternionStamped orient_utm;
  orient_tf.header.frame_id = this->frame_id_;
  orient_tf.header.stamp = ros::Time(0);
  orient_tf.quaternion.x = odom_msg->pose.pose.orientation.x;
  orient_tf.quaternion.y = odom_msg->pose.pose.orientation.y;
  orient_tf.quaternion.z = odom_msg->pose.pose.orientation.z;
  orient_tf.quaternion.w = odom_msg->pose.pose.orientation.w;
  try
  {
    this->listener_.transformQuaternion("utm", orient_tf, orient_utm);

  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }
  double roll, pitch, yaw;
  tf::Quaternion q_pose(orient_utm.quaternion.x, orient_utm.quaternion.y,
                        orient_utm.quaternion.z, orient_utm.quaternion.w);
  tf::Matrix3x3 m_pose(q_pose);
  m_pose.getRPY(roll, pitch, yaw);
  yaw = (yaw * 180.0) / PI;
  ///////////////////////////////////////////////////////////
  
  // pose in utm frame
  this->pose_.coordinates.at(0) = node_utm.point.x;
  this->pose_.coordinates.at(1) = node_utm.point.y;
  this->pose_.coordinates.at(2) = node_utm.point.z;
  this->pose_.coordinates.at(3) = yaw;
  this->pose_.matrix[0][0] = odom_msg->pose.covariance[0];
  this->pose_.matrix[1][1] = odom_msg->pose.covariance[7];
  this->pose_.matrix[2][2] = odom_msg->pose.covariance[14];
  this->pose_.matrix[3][3] = odom_msg->pose.covariance[35];
  
  this->flag_pose_ = true;

  this->alg_.unlock();
}
void GlobalPlanningAlgNode::cb_getGoalMsg(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
  this->alg_.lock();
  
  ///////////////////////////////////////////////////////////
  ///// TRANSFORM TO TF FARME
  geometry_msgs::PointStamped node_tf;
  geometry_msgs::PointStamped node_utm;
  node_tf.header.frame_id = this->frame_id_;
  node_tf.header.stamp = ros::Time(0); //ros::Time::now();
  node_tf.point.x = goal_msg->pose.position.x;
  node_tf.point.y = goal_msg->pose.position.y;
  node_tf.point.z = goal_msg->pose.position.z;
  try
  {
    this->listener_.transformPoint("utm", node_tf, node_utm);

  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }
  
  geometry_msgs::QuaternionStamped orient_tf;
  geometry_msgs::QuaternionStamped orient_utm;
  orient_tf.header.frame_id = this->frame_id_;
  orient_tf.header.stamp = ros::Time(0);
  orient_tf.quaternion.x = goal_msg->pose.orientation.x;
  orient_tf.quaternion.y = goal_msg->pose.orientation.y;
  orient_tf.quaternion.z = goal_msg->pose.orientation.z;
  orient_tf.quaternion.w = goal_msg->pose.orientation.w;
  try
  {
    this->listener_.transformQuaternion("utm", orient_tf, orient_utm);

  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }
  double roll, pitch, yaw;
  tf::Quaternion q_pose(orient_utm.quaternion.x, orient_utm.quaternion.y,
                        orient_utm.quaternion.z, orient_utm.quaternion.w);
  tf::Matrix3x3 m_pose(q_pose);
  m_pose.getRPY(roll, pitch, yaw);
  yaw = (yaw * 180.0) / PI;
  ///////////////////////////////////////////////////////////
  
  // goal in /utm frame
  this->global_goal_.coordinates.at(0) = node_utm.point.x;
  this->global_goal_.coordinates.at(1) = node_utm.point.y;
  this->global_goal_.coordinates.at(2) = node_utm.point.z;
  this->global_goal_.coordinates.at(3) = yaw;
  
  this->flag_goal_ = true;

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
  
  geometry_msgs::PointStamped node_tf;
  geometry_msgs::PointStamped node_utm;
  node_utm.header.frame_id = "utm";
  node_utm.header.stamp = ros::Time(0); //ros::Time::now();
  int size_n = this->st_nodes_.size();
  for (i = 0; i < size_n; i++)
  {
  
    node_utm.point.x = this->st_nodes_[i].coordinates[0];
    node_utm.point.y = this->st_nodes_[i].coordinates[1];
    node_utm.point.z = 0.0;
    //node_utm.transform.rotation = tf::createQuaternionMsgFromYaw(this->st_nodes_[1].coordinates[3]);
  
    try
    {
      this->listener_.transformPoint(this->frame_id_, node_utm, node_tf);

    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
      return 0;
    }
  
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = node_tf.point.x;
    marker.pose.position.y = node_tf.point.y;
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

int GlobalPlanningAlgNode::parseLinksToRosMarker(visualization_msgs::MarkerArray& marker_array)
{
  int i;

  visualization_msgs::Marker marker;

  marker.header.frame_id = this->frame_id_;
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "links";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::LINE_LIST;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the scale of the marker
  marker.scale.x = 0.1;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  geometry_msgs::PointStamped node_tf;
  geometry_msgs::PointStamped node_utm;
  node_utm.header.frame_id = "utm";
  node_utm.header.stamp = ros::Time(0); //ros::Time::now();
  int size_n = this->st_nodes_.size();
  marker.id = size_n;
  for (i = 0; i < size_n; i++)
  {
  
    int nl = this->st_nodes_[i].nodesConnected.size();
    geometry_msgs::Point point;
    for(int j = 0; j<nl; j++)
    {
        
      node_utm.point.x = this->st_nodes_[i].coordinates[0];
      node_utm.point.y = this->st_nodes_[i].coordinates[1];
      node_utm.point.z = 0.0;
      try
      {
        this->listener_.transformPoint(this->frame_id_, node_utm, node_tf);

      }
      catch (tf::TransformException& ex)
      {
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        return 0;
      }
      point.x = node_tf.point.x; 
      point.y = node_tf.point.y;
      point.z = 0.0;
      
      marker.points.push_back(point);

      int id_link = this->st_nodes_[i].nodesConnected[j];
      node_utm.point.x = this->st_nodes_[id_link].coordinates[0];
      node_utm.point.y = this->st_nodes_[id_link].coordinates[1];
      node_utm.point.z = 0.0;
      try
      {
        this->listener_.transformPoint(this->frame_id_, node_utm, node_tf);

      }
      catch (tf::TransformException& ex)
      {
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        return 0;
      }
      point.x = node_tf.point.x; 
      point.y = node_tf.point.y;
      point.z = 0.0;
      
      marker.points.push_back(point); 
    }
    marker_array.markers.push_back(marker);
    
  }

  return 0;
}

void GlobalPlanningAlgNode::fromUtmTransform(void)
{
  
  this->tf_to_utm_.header.frame_id = "utm";
  this->tf_to_utm_.child_frame_id = this->frame_id_;
  this->tf_to_utm_.header.stamp = ros::Time::now();
  this->tf_to_utm_.transform.translation.x = this->st_nodes_[0].coordinates[0];
  this->tf_to_utm_.transform.translation.y = this->st_nodes_[0].coordinates[1];
  this->tf_to_utm_.transform.translation.z = this->st_nodes_[0].coordinates[2];
  this->tf_to_utm_.transform.rotation = tf::createQuaternionMsgFromYaw(PI/2.0);
  
  this->broadcaster_.sendTransform(this->tf_to_utm_);
  
  return;
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < GlobalPlanningAlgNode > (argc, argv, "global_planning_alg_node");
}
