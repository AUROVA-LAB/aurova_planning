#include "local_planning_alg_node.h"

LocalPlanningAlgNode::LocalPlanningAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<LocalPlanningAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 20;//in [Hz]
  
  cvInitFont(&this->font_, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0);
  image_transport::ImageTransport it_(this->public_node_handle_);
  
  this->local_planning_ = new LocalPlanning();
  
  this->public_node_handle_.getParam("/local_planning/frame_lidar", this->frame_lidar_);
  
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
                                       
  lidar_config_.num_of_azimuth_cells = 1 + (lidar_config_.max_azimuth_angle - lidar_config_.min_azimuth_angle) / 
                                           lidar_config_.grid_azimuth_angular_resolution;
  lidar_config_.num_of_elevation_cells = 1 + (lidar_config_.max_elevation_angle - lidar_config_.min_elevation_angle) / 
                                           lidar_config_.grid_elevation_angular_resolution;
  
  
  
  // [init publishers]
  this->lidar_publisher_ = public_node_handle_.advertise < sensor_msgs::PointCloud2 > ("/velodyne_obstacles", 1);
  this->obstacles_publisher_ = public_node_handle_.advertise < sensor_msgs::PointCloud2 > ("/ground_obstacles", 1);
  this->plot_publisher_ = it_.advertise("/plot_pf_map", 1);
  
  // [init subscribers]
  this->lidar_subscriber_ = this->public_node_handle_.subscribe("/velodyne_points", 1,
                                                                &LocalPlanningAlgNode::cb_lidarInfo, this);
  
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
}

/*  [subscriber callbacks] */
void LocalPlanningAlgNode::cb_lidarInfo(const sensor_msgs::PointCloud2::ConstPtr& scan)
{
  this->alg_.lock();
  
  
  //////////////////////////////////////////////////
  //// free-space perimeter calculation
  sensor_msgs::PointCloud2 scan_filt;
  pcl::PCLPointCloud2 scan_pcl2;
  static pcl::PointCloud<pcl::PointXYZ> scan_pcl;
  static pcl::PointCloud<pcl::PointXYZ> scan_pcl_filt;
  static pcl::PointCloud<pcl::PointXYZ> free_space_pcl;

  pcl_conversions::toPCL(*scan, scan_pcl2);
  pcl::fromPCLPointCloud2(scan_pcl2, scan_pcl);
  
  this->local_planning_->freeSpaceMap(scan_pcl,
                                      this->lidar_config_,
                                      this->filter_config_,
                                      scan_pcl_filt,
                                      free_space_pcl);
                                      
  pcl::toPCLPointCloud2(scan_pcl_filt, scan_pcl2);
  pcl_conversions::fromPCL(scan_pcl2, scan_filt);
  
  scan_pcl_filt.header.frame_id = this->frame_lidar_;
  free_space_pcl.header.frame_id = this->frame_lidar_;
  this->lidar_publisher_.publish(scan_pcl_filt);
  this->obstacles_publisher_.publish(free_space_pcl);
  //////////////////////////////////////////////////
  
  
  //////////////////////////////////////////////////
  //// potential forces map calculation
  int offset = (int)(filter_config_.max_range); 
  int size = offset * 2;
  cv::Mat pf_map(size, size, CV_8UC3, EMPTY_PIXEL);
  this->alg_.potentialForcesMap(free_space_pcl, size, offset, pf_map);
  
  //plot
  std_msgs::Header header; // empty header
  header.stamp = ros::Time::now(); // time
  cv_bridge::CvImage output_bridge;
  output_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, pf_map);
  this->plot_publisher_.publish(output_bridge.toImageMsg());
  //////////////////////////////////////////////////
  
  this->alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void LocalPlanningAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void LocalPlanningAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<LocalPlanningAlgNode>(argc, argv, "local_planning_alg_node");
}
