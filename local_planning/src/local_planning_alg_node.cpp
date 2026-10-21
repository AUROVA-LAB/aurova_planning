#include "local_planning_alg_node.h"

LocalPlanningAlgNode::LocalPlanningAlgNode(void) :
    Node("local_planning")
{
  //init class attributes if necessary
  //this->loop_rate_ = 25; //in [Hz]

  
  this->goal_received_ = false;
  this->ctrl_received_ = false;

  this->pos_frame_.x = 0.0;
  this->pos_frame_.y = 0.0;
  this->pos_frame_.z = 0.0;

  this->declare_parameter("stop_code", 10.0);
  this->get_parameter("stop_code", this->stop_code_);

  this->declare_parameter("max_angle", 24.0);
  this->get_parameter("max_angle", this->ackermann_control_.max_angle);
  this->declare_parameter("delta_angle", 2.0);
  this->get_parameter("delta_angle", this->ackermann_control_.delta_angle);
  this->declare_parameter("v_length", 1.05);
  this->get_parameter("v_length", this->ackermann_control_.v_length);
  this->declare_parameter("delta_arc", 1.0);
  this->get_parameter("delta_arc", this->ackermann_control_.delta_arc);
  this->declare_parameter("max_arc", 2.0);
  this->get_parameter("max_arc", this->ackermann_control_.max_arc);
  this->declare_parameter("v_min", 0.4);
  this->get_parameter("v_min", this->ackermann_control_.v_min);
  this->declare_parameter("v_max", 1.3);
  this->get_parameter("v_max", this->ackermann_control_.v_max);
  this->declare_parameter("kp", 0.6);
  this->get_parameter("kp", this->ackermann_control_.kp);
  this->declare_parameter("margin", 1.0);
  this->get_parameter("margin", this->ackermann_control_.margin);
  this->declare_parameter("carrot_ctrl", false);
  this->get_parameter("carrot_ctrl", this->ackermann_control_.carrot_ctrl);
  this->declare_parameter("carrot_distance", 4.0);
  this->get_parameter("carrot_distance", this->ackermann_control_.carrot_distance);
  this->declare_parameter("carrot_stop_distance", 1.5);
  this->get_parameter("carrot_stop_distance", this->ackermann_control_.carrot_stop_distance);

  bool external_obstacle_detection;
  float wa, wr, aa, ar, wa2;
  this->declare_parameter("frame_id","map");
  this->get_parameter("frame_id", this->frame_id_);
  this->declare_parameter("frame_lidar", "os_sensor");
  this->get_parameter("frame_lidar", this->frame_lidar_);
  this->declare_parameter("save_data",false);
  this->get_parameter("save_data", this->save_data_);
  this->declare_parameter(
      "url_file_out",
      "/home/aurova/aurova-lab/aurova_ws/src/application_navigation/output_files/scans/");
  this->get_parameter("url_file_out", this->url_file_out_);
  this->declare_parameter("wa", 100.0);
  this->get_parameter("wa", wa);
  this->declare_parameter("wr", 1.0);
  this->get_parameter("wr", wr);
  this->declare_parameter("aa", 0.3);
  this->get_parameter("aa", aa);
  this->declare_parameter("ar", 0.8);
  this->get_parameter("ar", ar);
  this->declare_parameter("wa2", 3.0);
  this->get_parameter("wa2", wa2);
  this->declare_parameter("external_obstacle_detection", true);
  this->get_parameter("external_obstacle_detection", external_obstacle_detection);
  this->local_planning_ = new LocalPlanning(wa, wr, aa, ar, wa2);

  this->declare_parameter("max_range", 100.0);
  this->get_parameter("max_range", filter_config_.max_range);
  this->declare_parameter("min_range", 0.0);
  this->get_parameter("min_range", filter_config_.min_range);
  this->declare_parameter("a", 52.0);
  this->get_parameter("a", filter_config_.a);
  this->declare_parameter("b", 90.0);
  this->get_parameter("b", filter_config_.b);
  this->declare_parameter("c", -1.105);
  this->get_parameter("c", filter_config_.c);
  this->declare_parameter("lidar_protect", 1.0);
  this->get_parameter("lidar_protect", filter_config_.lidar_protect);
  this->declare_parameter("variance", 0.1);
  this->get_parameter("variance", filter_config_.variance);
  this->declare_parameter("radious", 8.0);
  this->get_parameter("radious", filter_config_.radious);
  this->declare_parameter("var_factor", 0.4);
  this->get_parameter("var_factor", filter_config_.var_factor);
  this->declare_parameter("ground_in_sim", -0.5);
  this->get_parameter("ground_in_sim", filter_config_.ground_in_sim);
  this->declare_parameter("is_simulation", false);
  this->get_parameter("is_simulation", filter_config_.is_simulation);
  this->declare_parameter("is_reconfig", false);
  this->get_parameter("is_reconfig", this->is_reconfig_);

  this->declare_parameter("max_elevation_angle", 112.5);
  this->get_parameter("max_elevation_angle", lidar_config_.max_elevation_angle);
  this->declare_parameter("min_elevation_angle", 67.5);
  this->get_parameter("min_elevation_angle", lidar_config_.min_elevation_angle);
  this->declare_parameter("max_azimuth_angle", 360.0);
  this->get_parameter("max_azimuth_angle", lidar_config_.max_azimuth_angle);
  this->declare_parameter("min_azimuth_angle", 0.0);
  this->get_parameter("min_azimuth_angle", lidar_config_.min_azimuth_angle);
  this->declare_parameter("grid_azimuth_angular_resolution", 0.18);
  this->get_parameter("grid_azimuth_angular_resolution",
                                     lidar_config_.grid_azimuth_angular_resolution);
  this->declare_parameter("grid_elevation_angular_resolution", 0.35);
  this->get_parameter("grid_elevation_angular_resolution",lidar_config_.grid_elevation_angular_resolution);

  lidar_config_.num_of_azimuth_cells = 1
      + (lidar_config_.max_azimuth_angle - lidar_config_.min_azimuth_angle)
          / lidar_config_.grid_azimuth_angular_resolution;
  lidar_config_.num_of_elevation_cells = 1
      + (lidar_config_.max_elevation_angle - lidar_config_.min_elevation_angle)
          / lidar_config_.grid_elevation_angular_resolution;

  // [init publishers]
  if (!external_obstacle_detection){
    this->obstacles_publisher_ = this->create_publisher < sensor_msgs::msg::PointCloud2 > ("/velodyne_obstacles", 1);
    this->ground_publisher_ = this->create_publisher < sensor_msgs::msg::PointCloud2 > ("/ground_obstacles", 1);
    this->limits_publisher_ = this->create_publisher < sensor_msgs::msg::PointCloud2 > ("/ground_limits", 1);
  }
  if (this->is_reconfig_)
    this->plane_publisher_ = this->create_publisher < sensor_msgs::msg::PointCloud2 > ("/ground_plane", 1);
  this->local_goal_publisher_ = this->create_publisher < sensor_msgs::msg::PointCloud2 > ("/local_goal", 1);
  this->collision_publisher_ = this->create_publisher < sensor_msgs::msg::PointCloud2 > ("/collision_risk", 1);
  this->collision_actions_publisher_ = this->create_publisher < sensor_msgs::msg::PointCloud2
      > ("/collision_actions", 1);
  this->free_actions_publisher_ = this->create_publisher < sensor_msgs::msg::PointCloud2 > ("/free_actions", 1);

  this->ackermann_rad_publisher_ = this->create_publisher < ackermann_msgs::msg::AckermannDrive> ("/ackermann_cmd", 1);
  this->ackermann_deg_publisher_ = this->create_publisher < ackermann_msgs::msg::AckermannDrive> ("/desired_ackermann_state", 1);

  // [init create_subscription<msgs::msg::YourMessageType>rs]
  if (!external_obstacle_detection){
    this->lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points", 1,
                              std::bind(&LocalPlanningAlgNode::cb_lidarInfo,this, std::placeholders::_1));
  }
  else{
    this->lidar_sync_subscriber_.subscribe(this,"/velodyne_points");
    this->obstacle_sync_subscriber_.subscribe(this,"/ouster_obstacles");
    this->ground_obstacle_sync_subscriber_.subscribe(this,"/ground_obstacles");
    this->ground_limits_sync_subscriber_.subscribe(this,"/ground_limits");
    this->sync = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(3), lidar_sync_subscriber_, obstacle_sync_subscriber_,
                                                            ground_obstacle_sync_subscriber_, ground_limits_sync_subscriber_);
    this->sync->registerCallback(std::bind(&LocalPlanningAlgNode::cb_lidarInfo2, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
  }
  
  this->goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/semilocal_goal", 1,
                            std::bind(&LocalPlanningAlgNode::cb_getGoalMsg,this, std::placeholders::_1));
  this->pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/pose_plot", 1, 
                            std::bind(&LocalPlanningAlgNode::cb_getPoseMsg,this, std::placeholders::_1));

  auto parameter_callback = this->add_on_set_parameters_callback(std::bind(&LocalPlanningAlgNode::node_config_update, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

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

  if (this->ctrl_received_)
  {
    this->ackermann_rad_publisher_->publish(this->ackermann_state_rad_);
    this->ackermann_deg_publisher_->publish(this->ackermann_state_deg_);
  }
}

/*  [create_subscription<msgs::msg::YourMessageType>r callbacks] */
void LocalPlanningAlgNode::cb_lidarInfo(const sensor_msgs::msg::PointCloud2::SharedPtr scan)
{
  double ini, end;

  ini = this->now().seconds();

  if (this->goal_received_)
  {

    //////////////////////////////////////////////////
    //// 1) free-space perimeter calculation
    pcl::PCLPointCloud2 scan_pcl2;
    static pcl::PointCloud<pcl::PointXYZ> scan_pcl;
    static pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pcl_filt = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    static pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_pcl = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    static pcl::PointCloud<pcl::PointXYZ>::Ptr limits_pcl = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl_conversions::toPCL(*scan, scan_pcl2);
    pcl::fromPCLPointCloud2(scan_pcl2, scan_pcl);

    this->local_planning_->groundSegmentation(scan_pcl, this->lidar_config_, this->filter_config_, *scan_pcl_filt,
                                              *obstacles_pcl, *limits_pcl);


    this->publishPointCloud(this->obstacles_publisher_, scan_pcl_filt, this->frame_lidar_, scan->header.stamp);
    this->publishPointCloud(this->ground_publisher_, obstacles_pcl, this->frame_lidar_, scan->header.stamp);
    this->publishPointCloud(this->limits_publisher_, limits_pcl, this->frame_lidar_, scan->header.stamp);
    //////////////////////////////////////////////////

    //////////////////////////////////////////////////
    //// 2) local goal calculation
    static pcl::PointCloud<pcl::PointXYZ>::Ptr local_path = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    this->local_planning_->localGoalCalculation(this->goal_lidar_, *obstacles_pcl, *limits_pcl, *local_path);

    //local_path.points.clear();
    //local_path.points.push_back(local_goal);
    local_path->points[local_path->points.size() - 2].x = local_path->points[local_path->points.size() - 2].x
        + this->base_in_lidarf_.x;
    local_path->points[local_path->points.size() - 2].y = local_path->points[local_path->points.size() - 2].y
        + this->base_in_lidarf_.y;
    this->publishPointCloud(this->local_goal_publisher_, local_path, this->frame_lidar_, scan->header.stamp);
    //////////////////////////////////////////////////

    //////////////////////////////////////////////////
    //// 3) control action calculation
    static pcl::PointCloud<pcl::PointXYZ>::Ptr collision_risk = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    static pcl::PointCloud<pcl::PointXYZ>::Ptr collision_actions = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    static pcl::PointCloud<pcl::PointXYZ>::Ptr free_actions = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    this->local_planning_->controlActionCalculation(local_path->points[local_path->points.size() - 2], this->goal_lidar_,
                                                    this->base_in_lidarf_, *scan_pcl_filt, *collision_risk,
                                                    *collision_actions, *free_actions, this->ackermann_control_);

    this->publishPointCloud(this->collision_publisher_, collision_risk, this->frame_lidar_, scan->header.stamp);
    this->publishPointCloud(this->collision_actions_publisher_, collision_actions, this->frame_lidar_, scan->header.stamp);
    this->publishPointCloud(this->free_actions_publisher_, free_actions, this->frame_lidar_, scan->header.stamp);


    this->ackermann_state_rad_.steering_angle = this->ackermann_control_.steering;
    this->ackermann_state_rad_.speed = this->ackermann_control_.velocity;

    //check is final position
    if (this->goal_lidar_.z == this->stop_code_)
    {
      this->ackermann_state_rad_.speed = 0.0;
      this->ackermann_state_deg_.speed = 0.0;
      this->ackermann_state_rad_.steering_angle = 0.0;
      this->ackermann_state_deg_.steering_angle = 0.0;
    }

    //proportional filtration
    static float speed_prev = 0.0;
    this->ackermann_state_rad_.speed = speed_prev
        + (this->ackermann_state_rad_.speed - speed_prev) * this->ackermann_control_.kp;
    speed_prev = this->ackermann_state_rad_.speed;

    this->ackermann_state_deg_ = this->ackermann_state_rad_;
    this->ackermann_state_deg_.steering_angle = this->ackermann_state_rad_.steering_angle * 180.0 / PI;
    this->ctrl_received_ = true;
    //////////////////////////////////////////////////

    // loop time
    end = this->now().seconds();
    RCLCPP_INFO(this->get_logger(),"duration total: %f", end - ini);

    //// SAVE DATA FOR PAPER
    if (this->save_data_)
    {
      static int cont = 0;
      static int subcont = 5;
      static int dw_factor = 5;

      std::ostringstream out_path_obs3d;
      std::ostringstream out_obs3d;
      std::ofstream file_obs3d;

      std::ostringstream out_path_obs2d;
      std::ostringstream out_obs2d;
      std::ofstream file_obs2d;

      std::ostringstream out_path_time;
      std::ostringstream out_time;
      std::ofstream file_time;

      std::ostringstream out_path_pos2d;
      std::ostringstream out_pos2d;
      std::ofstream file_pos2d;

      out_path_obs3d << this->url_file_out_ << cont << "_01_obs3d.csv";
      out_path_obs2d << this->url_file_out_ << cont << "_02_obs2d.csv";
      out_path_pos2d << this->url_file_out_ << cont << "_03_pos2d.csv";
      out_path_time << this->url_file_out_ << cont << "_04_time.csv";

      file_time.open(out_path_time.str().c_str(), std::ofstream::trunc);
      out_time << (end - ini) << "\n";
      file_time << out_time.str();
      file_time.close();

      file_pos2d.open(out_path_pos2d.str().c_str(), std::ofstream::trunc);
      // z is the variance of x and y
      out_pos2d << this->pos_frame_.x << ", " << this->pos_frame_.y << ", " << this->pos_frame_.z << "\n";
      file_pos2d << out_pos2d.str();
      file_pos2d.close();

      if (subcont == dw_factor)
      {
        subcont = 0;

        file_obs3d.open(out_path_obs3d.str().c_str(), std::ofstream::trunc);
        for (int i = 0; i < scan_pcl_filt->points.size(); i++)
        {
          out_obs3d << scan_pcl_filt->points[i].x << ", " << scan_pcl_filt->points[i].y << "\n";
        }
        file_obs3d << out_obs3d.str();
        file_obs3d.close();

        file_obs2d.open(out_path_obs2d.str().c_str(), std::ofstream::trunc);
        for (int i = 0; i < obstacles_pcl->points.size(); i++)
        {
          out_obs2d << obstacles_pcl->points[i].x << ", " << obstacles_pcl->points[i].y << "\n";
        }
        file_obs2d << out_obs2d.str();
        file_obs2d.close();
      }

      cont++;
      subcont++;
    }
    
    // This part is only for plane adjustment!!
    if (this->is_reconfig_){
      static pcl::PointCloud<pcl::PointXYZ>::Ptr plane_adj = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointXYZ point_p;

      int upper_limit = 400;
      int lower_limit = -400;
      float x_p, y_p, z_p;

      plane_adj->points.clear();

      for (int i = lower_limit; i < upper_limit; i++){
        for (int j = lower_limit; j < upper_limit; j++){
          x_p = (float)i / 100.0;
          y_p = (float)j / 100.0;
          z_p = -(x_p/this->filter_config_.a + y_p/this->filter_config_.b - 1) * this->filter_config_.c;

          point_p.x = x_p;
          point_p.y = y_p;
          point_p.z = z_p;
          plane_adj->points.push_back(point_p);
        }
      }
      this->publishPointCloud(this->plane_publisher_, plane_adj, this->frame_lidar_, scan->header.stamp);
    }

  }


}

void LocalPlanningAlgNode::cb_lidarInfo2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan,const sensor_msgs::msg::PointCloud2::ConstSharedPtr& obstacles, 
                    const sensor_msgs::msg::PointCloud2::ConstSharedPtr&  ground_obstacles, const sensor_msgs::msg::PointCloud2::ConstSharedPtr&  ground_limits)
{

  double ini, end;

  ini = this->now().seconds();

  if (this->goal_received_)
  {

    //////////////////////////////////////////////////
    //// 1) free-space perimeter calculation
    pcl::PCLPointCloud2 aux;
    static pcl::PointCloud<pcl::PointXYZ> scan_pcl;
    static pcl::PointCloud<pcl::PointXYZ> scan_pcl_filt;
    static pcl::PointCloud<pcl::PointXYZ> obstacles_pcl;
    static pcl::PointCloud<pcl::PointXYZ> limits_pcl;

    pcl_conversions::toPCL(*scan, aux);
    pcl::fromPCLPointCloud2(aux, scan_pcl);
    pcl_conversions::toPCL(*obstacles, aux);
    pcl::fromPCLPointCloud2(aux, scan_pcl_filt);
    pcl_conversions::toPCL(*ground_obstacles, aux);
    pcl::fromPCLPointCloud2(aux, obstacles_pcl);
    pcl_conversions::toPCL(*ground_limits, aux);
    pcl::fromPCLPointCloud2(aux, limits_pcl);

    //////////////////////////////////////////////////

    //////////////////////////////////////////////////
    //// 2) local goal calculation
    static pcl::PointCloud<pcl::PointXYZ>::Ptr local_path = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    this->local_planning_->localGoalCalculation(this->goal_lidar_, obstacles_pcl, limits_pcl, *local_path);

    //local_path.points.clear();
    //local_path.points.push_back(local_goal);
    local_path->points[local_path->points.size() - 2].x = local_path->points[local_path->points.size() - 2].x
        + this->base_in_lidarf_.x;
    local_path->points[local_path->points.size() - 2].y = local_path->points[local_path->points.size() - 2].y
        + this->base_in_lidarf_.y;
    this->publishPointCloud(this->local_goal_publisher_, local_path, this->frame_lidar_, scan->header.stamp);
    //////////////////////////////////////////////////

    //////////////////////////////////////////////////
    //// 3) control action calculation
    static pcl::PointCloud<pcl::PointXYZ>::Ptr collision_risk = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    static pcl::PointCloud<pcl::PointXYZ>::Ptr collision_actions = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    static pcl::PointCloud<pcl::PointXYZ>::Ptr free_actions = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    this->local_planning_->controlActionCalculation(local_path->points[local_path->points.size() - 2], this->goal_lidar_,
                                                    this->base_in_lidarf_, scan_pcl_filt, *collision_risk,
                                                    *collision_actions, *free_actions, this->ackermann_control_);

    this->publishPointCloud(this->collision_publisher_, collision_risk, this->frame_lidar_, scan->header.stamp);
    this->publishPointCloud(this->collision_actions_publisher_, collision_actions, this->frame_lidar_, scan->header.stamp);
    this->publishPointCloud(this->free_actions_publisher_, free_actions, this->frame_lidar_, scan->header.stamp);

    this->ackermann_state_rad_.steering_angle = this->ackermann_control_.steering;
    this->ackermann_state_rad_.speed = this->ackermann_control_.velocity;

    //check is final position
    if (this->goal_lidar_.z == this->stop_code_)
    {
      this->ackermann_state_rad_.speed = 0.0;
      this->ackermann_state_deg_.speed = 0.0;
      this->ackermann_state_rad_.steering_angle = 0.0;
      this->ackermann_state_deg_.steering_angle = 0.0;
    }

    //proportional filtration
    static float speed_prev = 0.0;
    this->ackermann_state_rad_.speed = speed_prev
        + (this->ackermann_state_rad_.speed - speed_prev) * this->ackermann_control_.kp;
    speed_prev = this->ackermann_state_rad_.speed;

    this->ackermann_state_deg_ = this->ackermann_state_rad_;
    this->ackermann_state_deg_.steering_angle = this->ackermann_state_rad_.steering_angle * 180.0 / PI;
    this->ctrl_received_ = true;
    //////////////////////////////////////////////////

    // loop time
    end = this->now().seconds();

    //// SAVE DATA FOR PAPER
    if (this->save_data_)
    {
      static int cont = 0;
      static int subcont = 5;
      static int dw_factor = 5;

      std::ostringstream out_path_obs3d;
      std::ostringstream out_obs3d;
      std::ofstream file_obs3d;

      std::ostringstream out_path_obs2d;
      std::ostringstream out_obs2d;
      std::ofstream file_obs2d;

      std::ostringstream out_path_time;
      std::ostringstream out_time;
      std::ofstream file_time;

      std::ostringstream out_path_pos2d;
      std::ostringstream out_pos2d;
      std::ofstream file_pos2d;

      out_path_obs3d << this->url_file_out_ << cont << "_01_obs3d.csv";
      out_path_obs2d << this->url_file_out_ << cont << "_02_obs2d.csv";
      out_path_pos2d << this->url_file_out_ << cont << "_03_pos2d.csv";
      out_path_time << this->url_file_out_ << cont << "_04_time.csv";

      file_time.open(out_path_time.str().c_str(), std::ofstream::trunc);
      out_time << (end - ini) << "\n";
      file_time << out_time.str();
      file_time.close();

      file_pos2d.open(out_path_pos2d.str().c_str(), std::ofstream::trunc);
      // z is the variance of x and y
      out_pos2d << this->pos_frame_.x << ", " << this->pos_frame_.y << ", " << this->pos_frame_.z << "\n";
      file_pos2d << out_pos2d.str();
      file_pos2d.close();

      if (subcont == dw_factor)
      {
        subcont = 0;

        file_obs3d.open(out_path_obs3d.str().c_str(), std::ofstream::trunc);
        for (int i = 0; i < scan_pcl_filt.points.size(); i++)
        {
          out_obs3d << scan_pcl_filt.points[i].x << ", " << scan_pcl_filt.points[i].y << "\n";
        }
        file_obs3d << out_obs3d.str();
        file_obs3d.close();

        file_obs2d.open(out_path_obs2d.str().c_str(), std::ofstream::trunc);
        for (int i = 0; i < obstacles_pcl.points.size(); i++)
        {
          out_obs2d << obstacles_pcl.points[i].x << ", " << obstacles_pcl.points[i].y << "\n";
        }
        file_obs2d << out_obs2d.str();
        file_obs2d.close();
      }

      cont++;
      subcont++;
    }

  }


}

void LocalPlanningAlgNode::cb_getGoalMsg(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr goal_msg)
{

  ///////////////////////////////////////////////////////////
  ///// TRANSFORM FROM TF TO LIDAR FARME
  geometry_msgs::msg::PointStamped goal_tf;
  geometry_msgs::msg::PointStamped goal_lidar;
  goal_tf.header.frame_id = this->frame_id_;
  goal_tf.header.stamp = rclcpp::Time(0); //this->now();
  goal_tf.point.x = goal_msg->pose.pose.position.x;
  goal_tf.point.y = goal_msg->pose.pose.position.y;
  goal_tf.point.z = goal_msg->pose.pose.position.z;
  try
  {
    this->tf_buffer_->transform( goal_tf,  goal_lidar, this->frame_lidar_);


  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(),"[draw_frames] TF exception:\n%s", ex.what());
    return;
  }
  ///////////////////////////////////////////////////////////

  this->goal_lidar_.x = goal_lidar.point.x;
  this->goal_lidar_.y = goal_lidar.point.y;
  this->goal_lidar_.z = goal_msg->pose.pose.position.z;

  this->goal_received_ = true;

  ///////////////////////////////////////////////////////////
  ///// TRANSFORM FROM BASE TO LIDAR FARME
  geometry_msgs::msg::PointStamped pose_base;
  geometry_msgs::msg::PointStamped pose_lidar;
  pose_base.header.frame_id = "base_link";
  pose_base.header.stamp = rclcpp::Time(0); //this->now();
  pose_base.point.x = 0.0;
  pose_base.point.y = 0.0;
  pose_base.point.z = 0.0;
  try
  {
    this->tf_buffer_->transform( pose_base,  pose_lidar, this->frame_lidar_);

  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(),"[draw_frames] TF exception:\n%s", ex.what());
    return;
  }
  geometry_msgs::msg::QuaternionStamped orient_base;
  geometry_msgs::msg::QuaternionStamped orient_lidar;
  orient_base.header.frame_id = "base_link";
  orient_base.header.stamp = rclcpp::Time(0);
  orient_base.quaternion.x = 0.0;
  orient_base.quaternion.y = 0.0;
  orient_base.quaternion.z = 0.0;
  orient_base.quaternion.w = 1.0;
  try
  {
    this->tf_buffer_->transform( orient_base, orient_lidar, this->frame_lidar_);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(),"[draw_frames] TF exception:\n%s", ex.what());
    return;
  }
  double roll, pitch, yaw;
  tf2::Quaternion q_pose(orient_lidar.quaternion.x, orient_lidar.quaternion.y, orient_lidar.quaternion.z,
                        orient_lidar.quaternion.w);
  tf2::Matrix3x3 m_pose(q_pose);
  m_pose.getRPY(roll, pitch, yaw);
  //yaw = (yaw * 180.0) / PI;

  this->base_in_lidarf_.x = pose_lidar.point.x;
  this->base_in_lidarf_.y = pose_lidar.point.y;
  this->base_in_lidarf_.yaw = yaw;
  ///////////////////////////////////////////////////////////


}

void LocalPlanningAlgNode::cb_getPoseMsg(const nav_msgs::msg::Odometry::SharedPtr  pose_msg)
{

  this->pos_frame_.x = pose_msg->pose.pose.position.x;
  this->pos_frame_.y = pose_msg->pose.pose.position.y;
  this->pos_frame_.z = pose_msg->pose.covariance[0];


}

void LocalPlanningAlgNode::publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, std::string frame, rclcpp::Time now)
{
    sensor_msgs::msg::PointCloud2::SharedPtr pc2_cloud(new sensor_msgs::msg::PointCloud2);

    pcl::toROSMsg(*point_cloud, *pc2_cloud);
    pc2_cloud->header.frame_id = frame;
    pc2_cloud->header.stamp = now;
    publisher->publish(*pc2_cloud);
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

rcl_interfaces::msg::SetParametersResult LocalPlanningAlgNode::node_config_update(const std::vector<rclcpp::Parameter> &parameters)
{

  double rate;
  this->get_parameter("rate", rate);
  if(rate!=this->rate)
  {
    this->rate = rate;
    this->loop_rate = std::make_shared<rclcpp::Rate>(this->rate);
  }
  this->get_parameter("a", filter_config_.a);
  this->get_parameter("b", filter_config_.b);
  this->get_parameter("c", filter_config_.c);
  this->get_parameter("variance", filter_config_.variance);
  this->get_parameter("radious", filter_config_.radious);
  this->get_parameter("var_factor", filter_config_.var_factor);
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;

}


/* main function */
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared< LocalPlanningAlgNode >();
	node->rate = node->declare_parameter<double>("rate", 10.0);
	node->loop_rate = std::make_shared<rclcpp::Rate>(node->rate);

	while (rclcpp::ok()) {
		node->mainNodeThread();
		rclcpp::spin_some(node);
		node->loop_rate->sleep();
	}
	rclcpp::shutdown();
	return 0;
}
