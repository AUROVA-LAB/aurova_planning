#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <Eigen/Dense>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <geometry_msgs/msg/pose.hpp>
#include <detection_msgs/msg/position_list.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>

#include <iostream>
#include <math.h>
#include <limits>
#include <chrono> 

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


using namespace Eigen;
using namespace sensor_msgs;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;



 //Consider points around this distance of the obstacle location.
struct obstacle_info{
  geometry_msgs::msg::Point relative_position; //The position of the obstacle respect the lidar frame.
  geometry_msgs::msg::Point global_position; //The global position, used to approximate the speed.
  Eigen::Vector3f speed, relative_speed; //Similar to before, the relative speed for moving the points respect lidar frame.
  int id;
  double last_time;
};

class DynamicObstaclePredictionNode : public rclcpp::Node{
  vector<obstacle_info> obstacles;
  geometry_msgs::msg::Pose robot_pose;

  float filter_radious;
  double keep_time;
  float time_per_distance, safety_margin, robot_speed;


  //Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  pc_filtered_pub; 
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  pc_obstXY_pub; 

  //Subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber;  
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr speed_subscriber;  
  message_filters::Subscriber<detection_msgs::msg::PositionList>obstacles_sub;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> ground_points_sub, obstacles_points_sub;
  public:
    std::shared_ptr<rclcpp::Rate> loop_rate;
    double rate;
    DynamicObstaclePredictionNode();
    void predict_pointcloud(PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud);
    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr ,const sensor_msgs::msg::PointCloud2::ConstSharedPtr ,const detection_msgs::msg::PositionList::ConstSharedPtr );
    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr  pose_msg);
    void speed_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr  ackermann_msg);
    void publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr, PointCloud::Ptr, std::string, rclcpp::Time);
};
///////////////////////////////////////callback

DynamicObstaclePredictionNode::DynamicObstaclePredictionNode(void) : 
  Node("dynamic_obstacle_prediction"){
  obstacles = vector<obstacle_info>(0);

  /// Load Parameters
  this->declare_parameter("filter_radious", 0.8); 
  this->get_parameter("filter_radious", filter_radious);
  this->declare_parameter("keep_time", 2.0); 
  this->get_parameter("keep_time", keep_time);
  this->declare_parameter("time_per_distance", 0.7); 
  this->get_parameter("time_per_distance", time_per_distance);
  this->declare_parameter("safety_margin", 0.2); 
  this->get_parameter("safety_margin", safety_margin);

  robot_pose = geometry_msgs::msg::Pose();
  robot_pose.orientation.w=1;

  //Init subscribers
  obstacles_sub.subscribe(this,"/carla/base_link/walker_list"); 
  ground_points_sub.subscribe(this,"/ground_obstacles"); 
  obstacles_points_sub.subscribe(this,"/ouster_obstacles"); 
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, detection_msgs::msg::PositionList> MySyncPolicy;
  static message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), ground_points_sub, obstacles_points_sub, obstacles_sub);
  sync.registerCallback(std::bind(&DynamicObstaclePredictionNode::callback,this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  pose_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/localization", 1, std::bind(&DynamicObstaclePredictionNode::pose_callback, this, std::placeholders::_1));
  speed_subscriber = this->create_subscription<ackermann_msgs::msg::AckermannDrive>("/carla/base_link/ackermann_cmd", 1, std::bind(&DynamicObstaclePredictionNode::speed_callback, this, std::placeholders::_1));

  //Init publishers 
  pc_filtered_pub = this->create_publisher<sensor_msgs::msg::PointCloud2> ("/ouster_prediction_obstacles", 1);
  pc_obstXY_pub   = this->create_publisher<sensor_msgs::msg::PointCloud2> ("/ground_prediction_obstacles", 1);
}

void DynamicObstaclePredictionNode::pose_callback(const nav_msgs::msg::Odometry::SharedPtr  pose_msg){
  this->robot_pose=pose_msg->pose.pose;
}

void DynamicObstaclePredictionNode::speed_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr  ackermann_msg){
  //Approximate the speed in the local axis X, using the steering angle as the angular angle.
  this->robot_speed=ackermann_msg->speed*cos(ackermann_msg->steering_angle);
}

void DynamicObstaclePredictionNode::callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr ground_obstacles_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr ouster_obstacles_msg, const detection_msgs::msg::PositionList::ConstSharedPtr obstacles_position_msg)
{
  if (ground_obstacles_msg == NULL) return;
  PointCloud::Ptr ground_obstacles_pcl(new PointCloud);
  PointCloud::Ptr ouster_obstacles_pcl(new PointCloud);

  pcl::fromROSMsg(*ground_obstacles_msg, *ground_obstacles_pcl);
  pcl::fromROSMsg(*ouster_obstacles_msg, *ouster_obstacles_pcl);

  double now = this->now().seconds();
  //Remove old obstacles
  for(int j=0;j<obstacles.size();j++){
    if(now-obstacles[j].last_time>keep_time){
      obstacles.erase(obstacles.begin()+j);
      j--;
    }
  }

  //Get position and speed of obstacles
  for(int i=0;i<obstacles_position_msg->ids.size();i++){
    int id=obstacles_position_msg->ids[i];
    bool new_obstacle=true;
    //Calucluate global position of the obstacle
    geometry_msgs::msg::Point global_position;
    // global_position.x = round((robot_pose.position.x + obstacles_position_msg->positions[i].x)*100.0)/100.0;
    // global_position.y = round((robot_pose.position.y + obstacles_position_msg->positions[i].y)*100.0)/100.0;
    global_position.x = robot_pose.position.x + obstacles_position_msg->positions[i].x;
    global_position.y = robot_pose.position.y + obstacles_position_msg->positions[i].y;
    //Apply rotation to get relative position
    geometry_msgs::msg::Point relative_position;
    Eigen::Vector3f p(obstacles_position_msg->positions[i].x,obstacles_position_msg->positions[i].y,obstacles_position_msg->positions[i].z);
    Eigen::Quaternionf q(robot_pose.orientation.w,robot_pose.orientation.x,robot_pose.orientation.y,robot_pose.orientation.z);
    p = q.conjugate() * p;
    relative_position.x = p[0]; relative_position.y=p[1];
    for(int j=0;j<obstacles.size();j++){
      if(id == obstacles[j].id){
        //Deal with 0 division.
        if(now-obstacles[j].last_time>=0.1){
          Eigen::Vector3f diff (global_position.x-obstacles[j].global_position.x, global_position.y-obstacles[j].global_position.y, 0);
          //Don't take into account small differences.
          if(abs(diff[0])<0.1) diff[0] = 0; 
          if(abs(diff[1])<0.1) diff[1] = 0;
          obstacles[j].speed=0.6*diff/(now-obstacles[j].last_time) + 0.4*obstacles[j].speed;
          obstacles[j].relative_speed=q.conjugate()*obstacles[j].speed;
          obstacles[j].global_position=global_position;
          obstacles[j].last_time=now;
        }
        obstacles[j].relative_position=relative_position;
        new_obstacle=false;
      }
    }
    if(new_obstacle){
      obstacle_info aux = {relative_position,global_position,Eigen::Vector3f(0,0,0),Eigen::Vector3f(0,0,0),id, now};
      obstacles.push_back(aux);
    }
  }

  //Predict obstacles position
  PointCloud::Ptr ground_cloud (new PointCloud);
  PointCloud::Ptr ouster_cloud (new PointCloud);
  predict_pointcloud(ground_obstacles_pcl,ground_cloud);
  predict_pointcloud(ouster_obstacles_pcl,ouster_cloud);

  // obstacles point cloud projection in XY plane  
  ground_cloud->is_dense = false;
  ground_cloud->width = (int) ground_cloud->points.size();
  ground_cloud->height = 1;
  publishPointCloud(pc_obstXY_pub,ground_cloud, ground_obstacles_msg->header.frame_id, ground_obstacles_msg->header.stamp);

  // obstacles cloud
  ouster_cloud->is_dense = false;
  ouster_cloud->width = (int) ouster_cloud->points.size();
  ouster_cloud->height = 1;
  publishPointCloud(pc_filtered_pub,ouster_cloud, ground_obstacles_msg->header.frame_id, ground_obstacles_msg->header.stamp);
}

void DynamicObstaclePredictionNode::predict_pointcloud(PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud){
  for(auto point:input_cloud->points){
    bool point_added=false;
    for(int i=0;i<obstacles.size();i++){
      if (sqrt(pow(point.x-obstacles[i].relative_position.x,2)+pow(point.y-obstacles[i].relative_position.y,2))<=filter_radious){
        //Calculate when they will cross each other.
        float t=0;
        if(abs(obstacles[i].relative_speed[0]-robot_speed)>0.01){
          t =  abs(obstacles[i].relative_position.x/(1.3-obstacles[i].relative_speed[0]))-safety_margin;
          if(t<0 or t>5.0) t=0;
        }
        //Move the point
        Eigen::Vector3f traslation = t*obstacles[i].relative_speed;
        auto new_point=point;
        new_point.x+=traslation[0]; new_point.y+=traslation[1];
        output_cloud->push_back(new_point);

        //Move the point, with a trail
        // for(float alpha=1.0;alpha>0.85;alpha-=0.05){
        //   Eigen::Vector3f traslation = alpha*d*time_per_distance*obstacles[i].speed;
        //   auto new_point=point;
        //   new_point.x+=traslation[0]; new_point.y+=traslation[1];
        //   output_cloud->push_back(new_point);
        // }
        point_added=true;
        break;
      }
    }
    if(!point_added)
      output_cloud->push_back(point);
  }
}

void DynamicObstaclePredictionNode::publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher, PointCloud::Ptr point_cloud,
    std::string frame, rclcpp::Time now )
{
    sensor_msgs::msg::PointCloud2::SharedPtr pc2_cloud(new sensor_msgs::msg::PointCloud2);

    pcl::toROSMsg(*point_cloud, *pc2_cloud);
    pc2_cloud->header.frame_id = frame;
    pc2_cloud->header.stamp = now;
    publisher->publish(*pc2_cloud);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node =std::make_shared<DynamicObstaclePredictionNode>();
  node->rate = node->declare_parameter<double>("rate", 10.0);
	node->loop_rate = std::make_shared<rclcpp::Rate>(node->rate);
  rclcpp::spin(node);
  return 0;
}
