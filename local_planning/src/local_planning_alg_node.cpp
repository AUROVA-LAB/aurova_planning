#include "local_planning_alg_node.h"

LocalPlanningAlgNode::LocalPlanningAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<LocalPlanningAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 25; //in [Hz]

  this->local_planning_ = new LocalPlanning();
  this->goal_received_ = false;
  this->ctrl_received_ = false;

  this->public_node_handle_.getParam("/ackermann_control/max_angle", this->ackermann_control_.max_angle);
  this->public_node_handle_.getParam("/ackermann_control/delta_angle", this->ackermann_control_.delta_angle);
  this->public_node_handle_.getParam("/ackermann_control/v_length", this->ackermann_control_.v_length);
  this->public_node_handle_.getParam("/ackermann_control/delta_arc", this->ackermann_control_.delta_arc);
  this->public_node_handle_.getParam("/ackermann_control/max_arc", this->ackermann_control_.max_arc);
  this->public_node_handle_.getParam("/ackermann_control/v_min", this->ackermann_control_.v_min);
  this->public_node_handle_.getParam("/ackermann_control/v_max", this->ackermann_control_.v_max);
  this->public_node_handle_.getParam("/ackermann_control/kp", this->ackermann_control_.kp);
  this->public_node_handle_.getParam("/ackermann_control/margin", this->ackermann_control_.margin);

  this->public_node_handle_.getParam("/local_planning/frame_id", this->frame_id_);
  this->public_node_handle_.getParam("/local_planning/frame_lidar", this->frame_lidar_);
  this->public_node_handle_.getParam("/local_planning/save_map", this->save_map_);
  this->public_node_handle_.getParam("/local_planning/out_path_map", this->out_path_map_);

  this->public_node_handle_.getParam("/filter_configuration/max_range", filter_config_.max_range);
  this->public_node_handle_.getParam("/filter_configuration/min_range", filter_config_.min_range);
  this->public_node_handle_.getParam("/filter_configuration/a", filter_config_.a);
  this->public_node_handle_.getParam("/filter_configuration/b", filter_config_.b);
  this->public_node_handle_.getParam("/filter_configuration/c", filter_config_.c);
  this->public_node_handle_.getParam("/filter_configuration/variance", filter_config_.variance);
  this->public_node_handle_.getParam("/filter_configuration/radious", filter_config_.radious);
  this->public_node_handle_.getParam("/filter_configuration/var_factor", filter_config_.var_factor);
  this->public_node_handle_.getParam("/filter_configuration/ground_in_sim", filter_config_.ground_in_sim);
  this->public_node_handle_.getParam("/filter_configuration/is_simulation", filter_config_.is_simulation);
  this->public_node_handle_.getParam("/filter_configuration/is_reconfig", this->is_reconfig_);

  this->public_node_handle_.getParam("/lidar_configuration/max_elevation_angle", lidar_config_.max_elevation_angle);
  this->public_node_handle_.getParam("/lidar_configuration/min_elevation_angle", lidar_config_.min_elevation_angle);
  this->public_node_handle_.getParam("/lidar_configuration/max_azimuth_angle", lidar_config_.max_azimuth_angle);
  this->public_node_handle_.getParam("/lidar_configuration/min_azimuth_angle", lidar_config_.min_azimuth_angle);
  this->public_node_handle_.getParam("/lidar_configuration/grid_azimuth_angular_resolution",
                                     lidar_config_.grid_azimuth_angular_resolution);
  this->public_node_handle_.getParam("/lidar_configuration/grid_elevation_angular_resolution",
                                     lidar_config_.grid_elevation_angular_resolution);

  lidar_config_.num_of_azimuth_cells = 1
      + (lidar_config_.max_azimuth_angle - lidar_config_.min_azimuth_angle)
          / lidar_config_.grid_azimuth_angular_resolution;
  lidar_config_.num_of_elevation_cells = 1
      + (lidar_config_.max_elevation_angle - lidar_config_.min_elevation_angle)
          / lidar_config_.grid_elevation_angular_resolution;

  // [init publishers]
  this->obstacles_publisher_ = public_node_handle_.advertise < sensor_msgs::PointCloud2 > ("/velodyne_obstacles", 1);
  this->ground_publisher_ = public_node_handle_.advertise < sensor_msgs::PointCloud2 > ("/ground_obstacles", 1);
  this->limits_publisher_ = public_node_handle_.advertise < sensor_msgs::PointCloud2 > ("/ground_limits", 1);
  this->local_goal_publisher_ = public_node_handle_.advertise < sensor_msgs::PointCloud2 > ("/local_goal", 1);
  this->collision_publisher_ = public_node_handle_.advertise < sensor_msgs::PointCloud2 > ("/collision_risk", 1);
  this->collision_actions_publisher_ = public_node_handle_.advertise < sensor_msgs::PointCloud2
      > ("/collision_actions", 1);
  this->free_actions_publisher_ = public_node_handle_.advertise < sensor_msgs::PointCloud2 > ("/free_actions", 1);

  this->ackermann_rad_publisher_ = this->public_node_handle_.advertise < ackermann_msgs::AckermannDrive
      > ("/ackermann_cmd", 1);
  this->ackermann_deg_publisher_ = this->public_node_handle_.advertise < ackermann_msgs::AckermannDriveStamped
      > ("/desired_ackermann_state", 1);
  this->ford_rec_publisher_ = this->public_node_handle_.advertise < std_msgs::Float32
      > ("/forward_recommended_velocity", 1);

  this->back_rec_publisher_ = this->public_node_handle_.advertise < std_msgs::Float32
      > ("/backward_recommended_velocity", 1);

  // [init subscribers]
  this->lidar_subscriber_ = this->public_node_handle_.subscribe("/velodyne_points", 1,
                                                                &LocalPlanningAlgNode::cb_lidarInfo, this);
  this->goal_subscriber_ = this->public_node_handle_.subscribe("/semilocal_goal", 1,
                                                               &LocalPlanningAlgNode::cb_getGoalMsg, this);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

LocalPlanningAlgNode::~LocalPlanningAlgNode(void)
{
  // [free dynamic memory]
}

void LocalPlanningAlgNode::mainNodeThread(void)
{
  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]

  // recommended speed bridge
  ford_rec_.data = 1000;
  back_rec_.data = 1000;
  this->ford_rec_publisher_.publish(this->ford_rec_);
  this->back_rec_publisher_.publish(this->back_rec_);

  if (this->ctrl_received_)
  {
    this->ackermann_rad_publisher_.publish(this->ackermann_state_rad_.drive);
    this->ackermann_deg_publisher_.publish(this->ackermann_state_deg_);
  }
}

/*  [subscriber callbacks] */
void LocalPlanningAlgNode::cb_lidarInfo(const sensor_msgs::PointCloud2::ConstPtr &scan)
{
  this->alg_.lock();

  double ini, end;

  ini = ros::Time::now().toSec();

  if (this->goal_received_)
  {

    //////////////////////////////////////////////////
    //// 1) free-space perimeter calculation
    pcl::PCLPointCloud2 scan_pcl2;
    static pcl::PointCloud<pcl::PointXYZ> scan_pcl;
    static pcl::PointCloud<pcl::PointXYZ> scan_pcl_filt;
    static pcl::PointCloud<pcl::PointXYZ> obstacles_pcl;
    static pcl::PointCloud<pcl::PointXYZ> limits_pcl;

    pcl_conversions::toPCL(*scan, scan_pcl2);
    pcl::fromPCLPointCloud2(scan_pcl2, scan_pcl);

    this->local_planning_->groundSegmentation(scan_pcl, this->lidar_config_, this->filter_config_, scan_pcl_filt,
                                              obstacles_pcl, limits_pcl);

    scan_pcl_filt.header.frame_id = this->frame_lidar_;
    obstacles_pcl.header.frame_id = this->frame_lidar_;
    limits_pcl.header.frame_id = this->frame_lidar_;
    this->obstacles_publisher_.publish(scan_pcl_filt);
    this->ground_publisher_.publish(obstacles_pcl);
    this->limits_publisher_.publish(limits_pcl);
    //////////////////////////////////////////////////

    //////////////////////////////////////////////////
    //// 2) local goal calculation
    pcl::PointXYZ local_goal;
    static pcl::PointCloud<pcl::PointXYZ> local_goal_plot;

    this->local_planning_->localGoalCalculation(this->goal_lidar_, obstacles_pcl, limits_pcl, local_goal);

    local_goal_plot.points.clear();
    local_goal_plot.points.push_back(local_goal);
    local_goal_plot.header.frame_id = this->frame_lidar_;
    this->local_goal_publisher_.publish(local_goal_plot);
    //////////////////////////////////////////////////

    //////////////////////////////////////////////////
    //// 3) control action calculation
    static pcl::PointCloud<pcl::PointXYZ> collision_risk;
    static pcl::PointCloud<pcl::PointXYZ> collision_actions;
    static pcl::PointCloud<pcl::PointXYZ> free_actions;

    this->local_planning_->controlActionCalculation(local_goal, this->base_in_lidarf_, scan_pcl_filt, collision_risk,
                                                    collision_actions, free_actions, this->ackermann_control_);

    collision_risk.header.frame_id = this->frame_lidar_;
    collision_actions.header.frame_id = this->frame_lidar_;
    free_actions.header.frame_id = this->frame_lidar_;
    this->collision_publisher_.publish(collision_risk);
    this->collision_actions_publisher_.publish(collision_actions);
    this->free_actions_publisher_.publish(free_actions);

    this->ackermann_state_rad_.drive.steering_angle = this->ackermann_control_.steering;
    this->ackermann_state_rad_.drive.speed = this->ackermann_control_.velocity;

    //proportional filtration
    static float speed_prev = 0.0;
    this->ackermann_state_rad_.drive.speed = speed_prev
        + (this->ackermann_state_rad_.drive.speed - speed_prev) * this->ackermann_control_.kp;
    speed_prev = this->ackermann_state_rad_.drive.speed;

    this->ackermann_state_deg_ = this->ackermann_state_rad_;
    this->ackermann_state_deg_.drive.steering_angle = this->ackermann_state_rad_.drive.steering_angle * 180.0 / PI;
    this->ctrl_received_ = true;
    //////////////////////////////////////////////////

  }

  // loop time
  end = ros::Time::now().toSec();
  ROS_INFO("duration total: %f", end - ini);

  this->alg_.unlock();
}

void LocalPlanningAlgNode::cb_getGoalMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &goal_msg)
{
  this->alg_.lock();

  ///////////////////////////////////////////////////////////
  ///// TRANSFORM FROM TF TO LIDAR FARME
  geometry_msgs::PointStamped goal_tf;
  geometry_msgs::PointStamped goal_lidar;
  goal_tf.header.frame_id = this->frame_id_;
  goal_tf.header.stamp = ros::Time(0); //ros::Time::now();
  goal_tf.point.x = goal_msg->pose.pose.position.x;
  goal_tf.point.y = goal_msg->pose.pose.position.y;
  goal_tf.point.z = goal_msg->pose.pose.position.z;
  try
  {
    this->listener_.transformPoint(this->frame_lidar_, goal_tf, goal_lidar);

  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }
  ///////////////////////////////////////////////////////////

  this->goal_lidar_.x = goal_lidar.point.x;
  this->goal_lidar_.y = goal_lidar.point.y;
  this->goal_lidar_.z = 0;

  this->goal_received_ = true;

  ///////////////////////////////////////////////////////////
  ///// TRANSFORM FROM BASE TO LIDAR FARME
  geometry_msgs::PointStamped pose_base;
  geometry_msgs::PointStamped pose_lidar;
  pose_base.header.frame_id = "base_link";
  pose_base.header.stamp = ros::Time(0); //ros::Time::now();
  pose_base.point.x = 0.0;
  pose_base.point.y = 0.0;
  pose_base.point.z = 0.0;
  try
  {
    this->listener_.transformPoint(this->frame_lidar_, pose_base, pose_lidar);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }
  geometry_msgs::QuaternionStamped orient_base;
  geometry_msgs::QuaternionStamped orient_lidar;
  orient_base.header.frame_id = "base_link";
  orient_base.header.stamp = ros::Time(0);
  orient_base.quaternion.x = 0.0;
  orient_base.quaternion.y = 0.0;
  orient_base.quaternion.z = 0.0;
  orient_base.quaternion.w = 1.0;
  try
  {
    this->listener_.transformQuaternion(this->frame_lidar_, orient_base, orient_lidar);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }
  double roll, pitch, yaw;
  tf::Quaternion q_pose(orient_lidar.quaternion.x, orient_lidar.quaternion.y, orient_lidar.quaternion.z,
                        orient_lidar.quaternion.w);
  tf::Matrix3x3 m_pose(q_pose);
  m_pose.getRPY(roll, pitch, yaw);
  //yaw = (yaw * 180.0) / PI;

  this->base_in_lidarf_.x = pose_lidar.point.x;
  this->base_in_lidarf_.y = pose_lidar.point.y;
  this->base_in_lidarf_.yaw = yaw;
  ///////////////////////////////////////////////////////////

  this->alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void LocalPlanningAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;

  if (this->is_reconfig_)
  {
    this->filter_config_.a = this->config_.a;
    this->filter_config_.b = this->config_.b;
    this->filter_config_.c = this->config_.c;

    this->filter_config_.variance = this->config_.variance;
    this->filter_config_.radious = this->config_.radious;
    this->filter_config_.var_factor = this->config_.var_factor;

  }

  this->alg_.unlock();
}

void LocalPlanningAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < LocalPlanningAlgNode > (argc, argv, "local_planning_alg_node");
}
