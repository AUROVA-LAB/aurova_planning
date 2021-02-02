#include "local_planning_alg_node.h"

LocalPlanningAlgNode::LocalPlanningAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<LocalPlanningAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 25; //in [Hz]

  cvInitFont(&this->font_, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0);
  image_transport::ImageTransport it_(this->public_node_handle_);

  this->local_planning_ = new LocalPlanning();
  this->goal_received_ = false;
  this->ctrl_received_ = false;

  this->public_node_handle_.getParam("/ackermann_control/max_angle", this->ctrl_config_.max_angle);
  this->public_node_handle_.getParam("/ackermann_control/delta_angle", this->ctrl_config_.delta_angle);
  this->public_node_handle_.getParam("/ackermann_control/delta_time", this->ctrl_config_.delta_time);
  this->public_node_handle_.getParam("/ackermann_control/v_length", this->ctrl_config_.v_length);
  this->public_node_handle_.getParam("/ackermann_control/v_min", this->ctrl_config_.v_min);
  this->public_node_handle_.getParam("/ackermann_control/v_max", this->ctrl_config_.v_max);
  this->public_node_handle_.getParam("/ackermann_control/margin_sec", this->ctrl_config_.margin_sec);
  this->public_node_handle_.getParam("/ackermann_control/flag_prop", this->flag_prop_);
  this->public_node_handle_.getParam("/ackermann_control/kp", this->kp_);

  this->public_node_handle_.getParam("/pf_configuration/threshold_grad", this->pf_config_.threshold_grad);
  this->public_node_handle_.getParam("/pf_configuration/scale", this->pf_config_.scale);
  this->public_node_handle_.getParam("/pf_configuration/wr", this->pf_config_.wr);
  this->public_node_handle_.getParam("/pf_configuration/ar", this->pf_config_.ar);
  this->public_node_handle_.getParam("/pf_configuration/wa", this->pf_config_.wa);
  this->public_node_handle_.getParam("/pf_configuration/aa", this->pf_config_.aa);
  this->public_node_handle_.getParam("/pf_configuration/rad_min", this->pf_config_.rad_min);
  this->public_node_handle_.getParam("/pf_configuration/rad_max", this->pf_config_.rad_max);

  this->public_node_handle_.getParam("/local_planning/frame_id", this->frame_id_);
  this->public_node_handle_.getParam("/local_planning/frame_lidar", this->frame_lidar_);
  this->public_node_handle_.getParam("/local_planning/save_map", this->save_map_);
  this->public_node_handle_.getParam("/local_planning/out_path_map", this->out_path_map_);

  this->public_node_handle_.getParam("/filter_configuration/max_range", filter_config_.max_range);
  this->public_node_handle_.getParam("/filter_configuration/min_range", filter_config_.min_range);
  this->public_node_handle_.getParam("/filter_configuration/min_dot_product_for_ground",
                                     filter_config_.min_dot_product_for_ground);
  this->public_node_handle_.getParam("/filter_configuration/max_ground_elevation",
                                     filter_config_.max_ground_elevation_angle_change_in_degrees);

  this->public_node_handle_.getParam("/lidar_configuration/max_elevation_angle", lidar_config_.max_elevation_angle);
  this->public_node_handle_.getParam("/lidar_configuration/min_elevation_angle", lidar_config_.min_elevation_angle);
  this->public_node_handle_.getParam("/lidar_configuration/max_azimuth_angle", lidar_config_.max_azimuth_angle);
  this->public_node_handle_.getParam("/lidar_configuration/min_azimuth_angle", lidar_config_.min_azimuth_angle);
  this->public_node_handle_.getParam("/lidar_configuration/sensor_height", lidar_config_.sensor_height);
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
  this->lidar_publisher_ = public_node_handle_.advertise < sensor_msgs::PointCloud2 > ("/velodyne_obstacles", 1);
  this->obstacles_publisher_ = public_node_handle_.advertise < sensor_msgs::PointCloud2 > ("/ground_obstacles", 1);
  this->plot_publisher_ = it_.advertise("/plot_pf_map", 1);
  this->ackermann_publisher_ = this->public_node_handle_.advertise < ackermann_msgs::AckermannDrive
      > ("/ackermann_cmd", 1);
  this->ackermann_publisher2_ = this->public_node_handle_.advertise < ackermann_msgs::AckermannDriveStamped
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
    this->ackermann_publisher_.publish(this->ackermann_state_.drive);
    this->ackermann_publisher2_.publish(this->ackermann_state2_);
  }
}

/*  [subscriber callbacks] */
void LocalPlanningAlgNode::cb_lidarInfo(const sensor_msgs::PointCloud2::ConstPtr &scan)
{
  this->alg_.lock();

  double ini, end, ini1, end1, ini2, end2;

  ini = ros::Time::now().toSec();
  ini1 = ros::Time::now().toSec();

  if (this->goal_received_)
  {

    //////////////////////////////////////////////////
    //// free-space perimeter calculation
    sensor_msgs::PointCloud2 scan_filt;
    pcl::PCLPointCloud2 scan_pcl2;
    static pcl::PointCloud<pcl::PointXYZ> scan_pcl;
    static pcl::PointCloud<pcl::PointXYZ> scan_pcl_filt;
    static pcl::PointCloud<pcl::PointXYZ> free_space_pcl;

    pcl_conversions::toPCL(*scan, scan_pcl2);
    pcl::fromPCLPointCloud2(scan_pcl2, scan_pcl);

    this->local_planning_->freeSpaceMap(scan_pcl, this->lidar_config_, this->filter_config_, scan_pcl_filt,
                                        free_space_pcl);

    pcl::toPCLPointCloud2(scan_pcl_filt, scan_pcl2);
    pcl_conversions::fromPCL(scan_pcl2, scan_filt);

    scan_pcl_filt.header.frame_id = this->frame_lidar_;
    free_space_pcl.header.frame_id = this->frame_lidar_;
    this->lidar_publisher_.publish(scan_pcl_filt);
    this->obstacles_publisher_.publish(free_space_pcl);
    //////////////////////////////////////////////////

    end1 = ros::Time::now().toSec();

    //////////////////////////////////////////////////
    //// potential forces map calculation
    // size map calculation
    float max_x = 0.0, min_x = 0.0, max_y = 0.0, min_y = 0.0;
    int max_x_int, min_x_int, max_y_int, min_y_int;
    for (int i = 0; i < free_space_pcl.points.size(); ++i)
    {
      if (free_space_pcl.points[i].x > max_x)
      {
        max_x = free_space_pcl.points[i].x;
      }
      if (free_space_pcl.points[i].x < min_x)
      {
        min_x = free_space_pcl.points[i].x;
      }
      if (free_space_pcl.points[i].y > max_y)
      {
        max_y = free_space_pcl.points[i].y;
      }
      if (free_space_pcl.points[i].y < min_y)
      {
        min_y = free_space_pcl.points[i].y;
      }
    }
    max_x_int = (int)(max_x * this->pf_config_.scale);
    min_x_int = (int)(min_x * this->pf_config_.scale);
    max_y_int = (int)(max_y * this->pf_config_.scale);
    min_y_int = (int)(min_y * this->pf_config_.scale);
    this->pf_config_.size_x = max_x_int - min_x_int + 1;
    this->pf_config_.size_y = max_y_int - min_y_int + 1;
    this->pf_config_.offset_x = -min_x_int;
    this->pf_config_.offset_y = -min_y_int;

    //vector<vector<cv::Point> > contour_plt;
    //cv::Mat pf_map(this->pf_config_.size_y, this->pf_config_.size_x, CV_8UC3, EMPTY_PIXEL);
    //cv::Mat pf_map_plt(this->pf_config_.size_y, this->pf_config_.size_x, CV_8UC3, EMPTY_PIXEL);
    //this->alg_.potentialForcesMap(free_space_pcl, this->goal_lidar_, this->pf_config_, contour_plt, pf_map);
    //cv::applyColorMap(pf_map, pf_map_plt, cv::COLORMAP_JET);
    //////////////////////////////////////////////////

    //////////////////////////////////////////////////
    //// ROAD MAP CALCULATION
    //cv::Mat roads_map(this->pf_config_.size_y, this->pf_config_.size_x, CV_8UC3, EMPTY_PIXEL);
    //cv::Mat roads_map_plot(this->pf_config_.size_y, this->pf_config_.size_x, CV_8UC3, EMPTY_PIXEL);
    //this->alg_.findTransitableAreas(pf_map, contour_plt, this->goal_lidar_, this->pf_config_, roads_map);
    //cv::applyColorMap(roads_map, roads_map_plot, cv::COLORMAP_JET);
    //////////////////////////////////////////////////

    ini2 = ros::Time::now().toSec();

    //////////////////////////////////////////////////
    //// NAIVE GEODESIC PATH CALCULATION
    cv::Mat plot_img(this->pf_config_.size_y, this->pf_config_.size_x, CV_8UC1, EMPTY_PIXEL);
    vector < vector<cv::Point> > contour;
    vector < cv::Point2d > goal_candidates;
    cv::Point2d local_goal;
    int radious;
    this->alg_.findLocalGoal(free_space_pcl, this->goal_lidar_, this->pf_config_, contour, radious, plot_img,
                             goal_candidates, local_goal);

    //plot
    cv::Point2d uv, uv2;
    uv.x = this->pf_config_.offset_x;
    uv.y = this->pf_config_.offset_y;
    cv::circle(plot_img, uv, radious, CV_RGB(MAX_PIXEL, EMPTY_PIXEL, EMPTY_PIXEL), 1);
    for (int i = 0; i < goal_candidates.size(); i++)
    {
      cv::circle(plot_img, goal_candidates[i], 1, CV_RGB(EMPTY_PIXEL, EMPTY_PIXEL, MAX_PIXEL), -1);
    }
    cv::circle(plot_img, local_goal, 2, CV_RGB(EMPTY_PIXEL, MAX_PIXEL, EMPTY_PIXEL), -1);
    //////////////////////////////////////////////////

    //////////////////////////////////////////////////
    //// CONTROL ACTIONS
    static float speed_prev = 0.0;
    this->alg_.findControlAction(free_space_pcl, local_goal, this->base_in_lidarf_, this->goal_lidar_, this->pf_config_,
                                 this->ctrl_config_, contour, this->ackermann_state_, plot_img);

    //proportional filtration
    if (this->flag_prop_)
    {
      this->ackermann_state_.drive.speed = speed_prev + (this->ackermann_state_.drive.speed - speed_prev) * this->kp_;
      speed_prev = this->ackermann_state_.drive.speed;
    }
    this->ackermann_state2_ = this->ackermann_state_;
    this->ackermann_state2_.drive.steering_angle = this->ackermann_state_.drive.steering_angle * 180.0 / PI;
    this->ctrl_received_ = true;
    //////////////////////////////////////////////////

    ///////////////////////////////////////////////////
    //// PLOT AND FILE OUTPUT
    std_msgs::Header header; // empty header
    header.stamp = ros::Time::now(); // time
    cv_bridge::CvImage output_bridge;
    output_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, plot_img);
    this->plot_publisher_.publish(output_bridge.toImageMsg());
    if (this->save_map_)
    {
      static int cont = 0;
      std::ostringstream out_path_map;
      out_path_map << this->out_path_map_ << cont << ".jpg";
      cv::imwrite(out_path_map.str(), plot_img);
      cont++;
    }
    ///////////////////////////////////////////////////

  }

  // loop time
  end = ros::Time::now().toSec();
  end2 = ros::Time::now().toSec();
  ROS_INFO("duration naive: %f", (end1 - ini1) + (end2 - ini2));
  ROS_INFO("duration total (to plot): %f", end - ini);

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
  yaw = (yaw * 180.0) / PI;

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
  this->config_.a;
  this->config_.b;
  this->config_.c;

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
