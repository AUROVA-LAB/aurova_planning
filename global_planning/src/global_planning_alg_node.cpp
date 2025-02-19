#include "global_planning_alg_node.h"

GlobalPlanningAlgNode::GlobalPlanningAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<GlobalPlanningAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 10; //in [Hz]

  //////////////////////////////////////////////////
  // get aplication parameters
  std::string url_path, type_dist, string_closed_loop;
  double var_x, var_y, var_z, var_w;
  this->vectors_size_ = 4;
  this->index_path_ = 0;
  this->flag_pose_ = false;
  this->flag_goal_ = false;
  this->init_loop_ = false;
  this->public_node_handle_.getParam("/global_planning/url_path", url_path);
  this->public_node_handle_.getParam("/global_planning/url_file_out", this->url_file_out_);
  this->public_node_handle_.getParam("/global_planning/save_data", this->save_data_);
  this->public_node_handle_.getParam("/global_planning/frame_id", this->frame_id_);
  this->public_node_handle_.getParam("/global_planning/var_x", var_x);
  this->public_node_handle_.getParam("/global_planning/var_y", var_y);
  this->public_node_handle_.getParam("/global_planning/var_z", var_z);
  this->public_node_handle_.getParam("/global_planning/var_w", var_w);
  this->public_node_handle_.getParam("/global_planning/type_dist", type_dist);
  this->public_node_handle_.getParam("/global_planning/rad_reached", this->rad_reached_);
  this->public_node_handle_.getParam("/global_planning/operation_mode", this->operation_mode_);
  this->public_node_handle_.getParam("/global_planning/stop_code", this->stop_code_);
  this->public_node_handle_.getParam("/global_planning/closed_loop", string_closed_loop);
  std::istringstream parse_closed_loop(string_closed_loop);
  int number;
  while (parse_closed_loop>>number)
    this->closed_loop_.push_back(number) ; 

  //////////////////////////////////////////////////
  // set covariance matrix
  std::vector < std::vector<double> > covariance = Util::newMatrix();
  covariance[0][0] = var_x;
  covariance[1][1] = var_y;
  covariance[2][2] = var_z;
  covariance[3][3] = var_w;

  //////////////////////////////////////////////////
  // class constructor for graph
  Util::Distances type_dist2 = Util::Euclidean; // TODO: from param
  this->graph_ = new Graph(url_path, covariance, type_dist2, this->rad_reached_);

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
  this->index_loop = 0;

  //////////////////////////////////////////////////
  // parse graph to a struct array
  this->st_nodes_ = this->graph_->getStructGraph();
  //this->fromUtmTransform();

  // [init publishers]
  this->marker_pub_ = this->public_node_handle_.advertise < visualization_msgs::MarkerArray > ("/visualization", 1);
  this->local_goal_pub_ = this->public_node_handle_.advertise < geometry_msgs::PoseWithCovarianceStamped
      > ("/semilocal_goal", 1);

  // [init subscribers]
  this->pose_subscriber_ = this->public_node_handle_.subscribe("/pose_plot", 1, &GlobalPlanningAlgNode::cb_getPoseMsg,
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
    this->parseNodesToRosMarker(this->marker_array_);
    this->parseLinksToRosMarker(this->marker_array_);
    if (this->marker_array_.markers.size() > 0) first_exec = false;
  }

  switch (this->operation_mode_)
  {
    case MODE_PATH:

      if (this->flag_pose_ && this->flag_goal_)
      {
        float diff_x = this->st_path_[index_path_].coordinates[0] - this->pose_.coordinates[0];
        float diff_y = this->st_path_[index_path_].coordinates[1] - this->pose_.coordinates[1];
        float distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2));

        if ((distance < this->rad_reached_) && (index_path_ + 1 < this->st_path_.size()))
        {
          index_path_ = index_path_ + 1;
        }

        ///////////////////////////////////////////////////////////
        ///// TRANSFORM TO TF FARME
        geometry_msgs::PointStamped node_tf;
        geometry_msgs::PointStamped node_utm;
        node_utm.header.frame_id = "utm";
        node_utm.header.stamp = ros::Time(0); //ros::Time::now();
        node_utm.point.x = this->st_path_[index_path_].coordinates[0];
        node_utm.point.y = this->st_path_[index_path_].coordinates[1];
        node_utm.point.z = 0.0;
        try
        {
          this->listener_.transformPoint(this->frame_id_, node_utm, node_tf);
        }
        catch (tf::TransformException &ex)
        {
          ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
          return;
        }
        double yaw = this->getYawFromPath(index_path_);
        tf::Quaternion quat_utm = tf::createQuaternionFromRPY(0, 0, yaw);
        geometry_msgs::QuaternionStamped orient_tf;
        geometry_msgs::QuaternionStamped orient_utm;
        orient_utm.header.frame_id = "utm";
        orient_utm.header.stamp = ros::Time(0);
        orient_utm.quaternion.x = quat_utm[0];
        orient_utm.quaternion.y = quat_utm[1];
        orient_utm.quaternion.z = quat_utm[2];
        orient_utm.quaternion.w = quat_utm[3];
        try
        {
          this->listener_.transformQuaternion(this->frame_id_, orient_utm, orient_tf);

        }
        catch (tf::TransformException &ex)
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
        this->local_goal_.pose.covariance[0] = this->global_goal_.matrix[0][0];
        this->local_goal_.pose.covariance[7] = this->global_goal_.matrix[1][1];
        this->local_goal_.pose.covariance[14] = this->global_goal_.matrix[2][2];
        this->local_goal_.pose.covariance[35] = this->global_goal_.matrix[3][3];

        if ((distance < this->rad_reached_) && (index_path_ + 1 == this->st_path_.size()))
        {
          this->local_goal_.pose.pose.position.z = this->stop_code_;
        }

        this->local_goal_pub_.publish(this->local_goal_);
      }
      break;

    case MODE_LOOP:
      if (this->flag_pose_ && this->flag_goal_)
      {
        if (!this->init_loop_)
        {
          float min_dist = 100000;

          for (int i = 0; i < this->closed_loop_.size(); i++)
          {
            int index = this->closed_loop_[i];
            float diff_x = this->st_nodes_[index].coordinates[0] - this->pose_.coordinates[0];
            float diff_y = this->st_nodes_[index].coordinates[1] - this->pose_.coordinates[1];
            float distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2));

            if (distance < min_dist)
            {
              min_dist = distance;
              this->index_loop = i;
            }
          }
          this->init_loop_ = true;
        }

        ///////////////////////////////////////////////////////////
        ///// TRANSFORM TO TF FARME
        geometry_msgs::PointStamped node_tf;
        geometry_msgs::PointStamped node_utm;
        node_utm.header.frame_id = "utm";
        node_utm.header.stamp = ros::Time(0); //ros::Time::now();
        node_utm.point.x = this->st_nodes_[this->closed_loop_[this->index_loop]].coordinates[0];
        node_utm.point.y = this->st_nodes_[this->closed_loop_[this->index_loop]].coordinates[1];
        node_utm.point.z = 0.0;
        try
        {
          this->listener_.transformPoint(this->frame_id_, node_utm, node_tf);
        }
        catch (tf::TransformException &ex)
        {
          ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
          return;
        }
        // no yaw
        ///////////////////////////////////////////////////////////

        this->local_goal_.header.frame_id = this->frame_id_;
        this->local_goal_.pose.pose.position.x = node_tf.point.x;
        this->local_goal_.pose.pose.position.y = node_tf.point.y;
        this->local_goal_.pose.pose.position.z = 0.0;
        this->local_goal_.pose.pose.orientation.x = 0.0;
        this->local_goal_.pose.pose.orientation.y = 0.0;
        this->local_goal_.pose.pose.orientation.z = 0.0;
        this->local_goal_.pose.pose.orientation.w = 1.0;
        this->local_goal_.pose.covariance[0] = this->global_goal_.matrix[0][0];
        this->local_goal_.pose.covariance[7] = this->global_goal_.matrix[1][1];
        this->local_goal_.pose.covariance[14] = this->global_goal_.matrix[2][2];
        this->local_goal_.pose.covariance[35] = this->global_goal_.matrix[3][3];

        float diff_x = this->st_nodes_[this->closed_loop_[this->index_loop]].coordinates[0] - this->pose_.coordinates[0];
        float diff_y = this->st_nodes_[this->closed_loop_[this->index_loop]].coordinates[1] - this->pose_.coordinates[1];
        float distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
    
        if (distance < this->rad_reached_)
        {
          if (this->index_loop + 1 < this->closed_loop_.size())
          {
            this->index_loop = this->index_loop + 1;
          }
          else
          {
            this->index_loop = 0;
          }
        }

        this->local_goal_pub_.publish(this->local_goal_);
      }
      break;
    case MODE_BYPASS:
      if (this->flag_pose_ && this->flag_goal_)
      {
        this->local_goal_.header.frame_id = this->frame_id_;
        this->local_goal_.pose.pose.position.x = this->global_goal_bypass_.pose.position.x;
        this->local_goal_.pose.pose.position.y = this->global_goal_bypass_.pose.position.y;
        this->local_goal_.pose.pose.position.z = this->global_goal_bypass_.pose.position.z;
        this->local_goal_.pose.pose.orientation.x = this->global_goal_bypass_.pose.orientation.x;
        this->local_goal_.pose.pose.orientation.y = this->global_goal_bypass_.pose.orientation.y;
        this->local_goal_.pose.pose.orientation.z = this->global_goal_bypass_.pose.orientation.z;
        this->local_goal_.pose.pose.orientation.w = this->global_goal_bypass_.pose.orientation.w;
        this->local_goal_.pose.covariance[0] = this->global_goal_.matrix[0][0];
        this->local_goal_.pose.covariance[7] = this->global_goal_.matrix[1][1];
        this->local_goal_.pose.covariance[14] = this->global_goal_.matrix[2][2];
        this->local_goal_.pose.covariance[35] = this->global_goal_.matrix[3][3];

        this->local_goal_pub_.publish(this->local_goal_);
      }
      break;

    case MODE_INIT_LOOP:
      if (this->flag_pose_ && this->flag_goal_){
        this->index_loop = 0;
        this->init_loop_ = true;
        this->operation_mode_= MODE_LOOP;
        break;
      }
  }

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  //this->tf_to_utm_.header.seq = this->tf_to_utm_.header.seq + 1;
  //this->tf_to_utm_.header.stamp = ros::Time::now();
  //this->broadcaster_.sendTransform(this->tf_to_utm_);
  this->marker_pub_.publish(this->marker_array_);
}

/*  [subscriber callbacks] */
void GlobalPlanningAlgNode::cb_getPoseMsg(const nav_msgs::Odometry::ConstPtr& pose_msg)
{
  //this->alg_.lock();

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
  catch (tf::TransformException &ex)
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
  catch (tf::TransformException &ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }
  double roll, pitch, yaw;
  tf::Quaternion q_pose(orient_utm.quaternion.x, orient_utm.quaternion.y, orient_utm.quaternion.z,
                        orient_utm.quaternion.w);
  tf::Matrix3x3 m_pose(q_pose);
  m_pose.getRPY(roll, pitch, yaw);
  yaw = (yaw * 180.0) / PI;
///////////////////////////////////////////////////////////

// pose in utm frame
  this->pose_.coordinates.at(0) = node_utm.point.x;
  this->pose_.coordinates.at(1) = node_utm.point.y;
  this->pose_.coordinates.at(2) = this->st_nodes_[0].coordinates[2]; //node_utm.point.z;
  this->pose_.coordinates.at(3) = yaw;
  this->pose_.matrix[0][0] = pose_msg->pose.covariance[0];
  this->pose_.matrix[1][1] = pose_msg->pose.covariance[7];
  this->pose_.matrix[2][2] = pose_msg->pose.covariance[14];
  this->pose_.matrix[3][3] = pose_msg->pose.covariance[35];

  this->flag_pose_ = true;

  //this->alg_.unlock();
}

void GlobalPlanningAlgNode::cb_getGoalMsg(const geometry_msgs::PoseStamped::ConstPtr &goal_msg)
{
  //this->alg_.lock();

  this->init_loop_ = false;

  if (this->operation_mode_ == MODE_LOOP){
    std::string string_closed_loop;
    this->public_node_handle_.getParam("/global_planning/closed_loop", string_closed_loop);
    std::istringstream parse_closed_loop(string_closed_loop);
    int number;
    this->closed_loop_.clear();
    while (parse_closed_loop>>number)
      this->closed_loop_.push_back(number) ; 
  }
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
  catch (tf::TransformException &ex)
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
  catch (tf::TransformException &ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }
  double roll, pitch, yaw;
  tf::Quaternion q_pose(orient_utm.quaternion.x, orient_utm.quaternion.y, orient_utm.quaternion.z,
                        orient_utm.quaternion.w);
  tf::Matrix3x3 m_pose(q_pose);
  m_pose.getRPY(roll, pitch, yaw);
  yaw = (yaw * 180.0) / PI;
///////////////////////////////////////////////////////////

// goal in /utm frame
  this->global_goal_.coordinates.at(0) = node_utm.point.x;
  this->global_goal_.coordinates.at(1) = node_utm.point.y;
  this->global_goal_.coordinates.at(2) = node_utm.point.z;
  this->global_goal_.coordinates.at(3) = yaw;

  this->global_goal_bypass_.pose.position.x = goal_msg->pose.position.x;
  this->global_goal_bypass_.pose.position.y = goal_msg->pose.position.y;
  this->global_goal_bypass_.pose.position.z = goal_msg->pose.position.z;
  this->global_goal_bypass_.pose.orientation.x = goal_msg->pose.orientation.x;
  this->global_goal_bypass_.pose.orientation.y = goal_msg->pose.orientation.y;
  this->global_goal_bypass_.pose.orientation.z = goal_msg->pose.orientation.z;
  this->global_goal_bypass_.pose.orientation.w = goal_msg->pose.orientation.w;

  if (this->flag_pose_ && this->operation_mode_ == MODE_PATH)
  {
    visualization_msgs::MarkerArray marker_array;
    ROS_INFO("******** pre-execution ********");
    this->st_path_ = this->graph_->getPathPoses(this->pose_, this->global_goal_);
    ROS_INFO("******** pos-execution ********");
    this->parsePathToRosMarker(marker_array);
    this->marker_pub_.publish(marker_array);
    this->index_path_ = 0;
  }

  this->flag_goal_ = true;

  //this->alg_.unlock();
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

int GlobalPlanningAlgNode::parseNodesToRosMarker(visualization_msgs::MarkerArray &marker_array)
{
  int i;

  visualization_msgs::Marker marker;
  visualization_msgs::Marker text;

  marker.header.frame_id = this->frame_id_;
  marker.header.stamp = ros::Time::now();
  text.header.frame_id = this->frame_id_;
  text.header.stamp = ros::Time::now();

// Set the namespace and id for this marker.  This serves to create a unique ID
// Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "nodes";
  text.ns = "text";

// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::SPHERE;
  text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
  text.action = visualization_msgs::Marker::ADD;

// Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.7;
  marker.scale.y = 0.7;
  marker.scale.z = 0.05;

  text.scale.x = 0.7;
  text.scale.y = 0.7;
  text.scale.z = 0.7;

// Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  text.color.r = 0.0f;
  text.color.g = 0.0f;
  text.color.b = 0.0f;
  text.color.a = 0.7;

  marker.lifetime = ros::Duration();
  text.lifetime = ros::Duration();

  std::ostringstream out_nodes; // File out stream

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
    catch (tf::TransformException &ex)
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

    stringstream num;
    num << this->st_nodes_[i].id;
    text.text = num.str();
    text.pose.position.x = node_tf.point.x;
    text.pose.position.y = node_tf.point.y;
    text.pose.position.z = 1.0;

    marker.id = this->st_nodes_[i].id;
    text.id = this->st_nodes_[i].id;
    marker_array.markers.push_back(marker);
    marker_array.markers.push_back(text);

    if (this->save_data_)
      out_nodes << this->st_nodes_[i].id << ", " << node_tf.point.x << ", " << node_tf.point.y << "\n";

  }

  if (this->save_data_)
  {
    std::ostringstream out_nodes_path;
    std::ofstream file_nodes;

    out_nodes_path << this->url_file_out_ << "nodes.csv";

    file_nodes.open(out_nodes_path.str().c_str(), std::ofstream::trunc);
    file_nodes << out_nodes.str();
    file_nodes.close();
  }

  return 0;
}

int GlobalPlanningAlgNode::parsePathToRosMarker(visualization_msgs::MarkerArray &marker_array)
{
  int i;

  visualization_msgs::Marker marker;

  marker.header.frame_id = this->frame_id_;
  marker.header.stamp = ros::Time::now();

// Set the namespace and id for this marker.  This serves to create a unique ID
// Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "path";

// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::SPHERE;

// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

// Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.6;
  marker.scale.y = 0.6;
  marker.scale.z = 0.6;

// Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  geometry_msgs::PointStamped node_tf;
  geometry_msgs::PointStamped node_utm;
  node_utm.header.frame_id = "utm";
  node_utm.header.stamp = ros::Time(0); //ros::Time::now();
  int size_n = this->st_path_.size();
  for (i = 0; i < size_n; i++)
  {
    node_utm.point.x = this->st_path_[i].coordinates[0];
    node_utm.point.y = this->st_path_[i].coordinates[1];
    node_utm.point.z = 0.0;
    try
    {
      this->listener_.transformPoint(this->frame_id_, node_utm, node_tf);

    }
    catch (tf::TransformException &ex)
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

    marker.id = 1000 + i;
    marker_array.markers.push_back(marker);

  }

  return 0;
}

int GlobalPlanningAlgNode::parseLinksToRosMarker(visualization_msgs::MarkerArray &marker_array)
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
  marker.scale.x = 0.5;

// Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  std::ostringstream out_links; // File out stream

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
    for (int j = 0; j < nl; j++)
    {

      node_utm.point.x = this->st_nodes_[i].coordinates[0];
      node_utm.point.y = this->st_nodes_[i].coordinates[1];
      node_utm.point.z = 0.0;
      try
      {
        this->listener_.transformPoint(this->frame_id_, node_utm, node_tf);

      }
      catch (tf::TransformException &ex)
      {
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        return 0;
      }
      point.x = node_tf.point.x;
      point.y = node_tf.point.y;
      point.z = 0.0;

      marker.points.push_back(point);

      if (this->save_data_)
        out_links << this->st_nodes_[i].id << ", " << node_tf.point.x << ", " << node_tf.point.y;

      int id_link = this->st_nodes_[i].nodesConnected[j];
      node_utm.point.x = this->st_nodes_[id_link].coordinates[0];
      node_utm.point.y = this->st_nodes_[id_link].coordinates[1];
      node_utm.point.z = 0.0;
      try
      {
        this->listener_.transformPoint(this->frame_id_, node_utm, node_tf);

      }
      catch (tf::TransformException &ex)
      {
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        return 0;
      }
      point.x = node_tf.point.x;
      point.y = node_tf.point.y;
      point.z = 0.0;

      marker.points.push_back(point);

      if (this->save_data_)
        out_links << ", " << id_link << ", " << node_tf.point.x << ", " << node_tf.point.y << "\n";
    }
    marker_array.markers.push_back(marker);
  }

  if (this->save_data_)
  {
    std::ostringstream out_links_path;
    std::ofstream file_links;

    out_links_path << this->url_file_out_ << "links.csv";

    file_links.open(out_links_path.str().c_str(), std::ofstream::trunc);
    file_links << out_links.str();
    file_links.close();
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
  this->tf_to_utm_.transform.rotation = tf::createQuaternionMsgFromYaw(PI / 2.0);

  this->broadcaster_.sendTransform(this->tf_to_utm_);

  return;
}

double GlobalPlanningAlgNode::getYawFromPath(int index_path)
{
  double yaw;

  if (index_path + 1 == this->st_path_.size())
  {
    yaw = this->st_path_[index_path].coordinates[3];
    yaw = (yaw * PI) / 180.0;
  }
  else
  {
    float diff_x = this->st_path_[index_path + 1].coordinates[0] - this->st_path_[index_path].coordinates[0];
    float diff_y = this->st_path_[index_path + 1].coordinates[1] - this->st_path_[index_path].coordinates[1];
    yaw = atan2(diff_y, diff_x);
  }

  return yaw;
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < GlobalPlanningAlgNode > (argc, argv, "global_planning_alg_node");
}
