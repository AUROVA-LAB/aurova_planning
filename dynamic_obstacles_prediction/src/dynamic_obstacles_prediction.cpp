#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Dense>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Pose.h>
#include <detection_msgs/PositionList.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDrive.h>

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
  geometry_msgs::Point relative_position; //The position of the obstacle respect the lidar frame.
  geometry_msgs::Point global_position; //The global position, used to approximate the speed.
  Eigen::Vector3f speed, relative_speed; //Similar to before, the relative speed for moving the points respect lidar frame.
  int id;
  double last_time;
};

class DynamicObstaclePredictionNode{
  vector<obstacle_info> obstacles;
  geometry_msgs::Pose robot_pose;

  float filter_radious;
  double keep_time;
  float time_per_distance, safety_margin, robot_speed;
  ros::NodeHandle nh;

  //Publisher
  ros::Publisher pc_filtered_pub; 
  ros::Publisher pc_obstXY_pub; 

  //Subscriber
  ros::Subscriber pose_subscriber;  
  ros::Subscriber speed_subscriber;  
  message_filters::Subscriber<detection_msgs::PositionList>obstacles_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> ground_points_sub, obstacles_points_sub;
  public:
    DynamicObstaclePredictionNode();
    void predict_pointcloud(PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud);
    void callback(const sensor_msgs::PointCloud2::ConstPtr&,const sensor_msgs::PointCloud2::ConstPtr&,const detection_msgs::PositionListConstPtr&);
    void pose_callback(const nav_msgs::Odometry::ConstPtr& pose_msg);
    void speed_callback(const ackermann_msgs::AckermannDrive::ConstPtr& ackermann_msg);
};
///////////////////////////////////////callback

DynamicObstaclePredictionNode::DynamicObstaclePredictionNode(){
  obstacles = vector<obstacle_info>(0);
  
  nh = ros::NodeHandle("~");  
  /// Load Parameters
  nh.param("filter_radious", filter_radious, float(0.8));
  nh.param("keep_time", keep_time, 2.0);
  nh.param("time_per_distance", time_per_distance, float(0.7));
  nh.param("safety_margin", safety_margin, float(0.2));

  robot_pose = geometry_msgs::Pose();
  robot_pose.orientation.w=1;

  //Init subscribers
  obstacles_sub.subscribe(nh,"/carla/base_link/walker_list", 10); 
  ground_points_sub.subscribe(nh,"/ground_obstacles", 10); 
  obstacles_points_sub.subscribe(nh,"/ouster_obstacles", 10); 
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, detection_msgs::PositionList> MySyncPolicy;
  static message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), ground_points_sub, obstacles_points_sub, obstacles_sub);
  sync.registerCallback(boost::bind(&DynamicObstaclePredictionNode::callback,this, _1, _2, _3));

  pose_subscriber = nh.subscribe("/localization", 1, &DynamicObstaclePredictionNode::pose_callback, this);
  speed_subscriber = nh.subscribe("/carla/base_link/ackermann_cmd", 1, &DynamicObstaclePredictionNode::speed_callback, this);

  //Init publishers 
  pc_filtered_pub = nh.advertise<sensor_msgs::PointCloud2> ("/ouster_prediction_obstacles", 1);
  pc_obstXY_pub   = nh.advertise<sensor_msgs::PointCloud2> ("/ground_prediction_obstacles", 1);
}

void DynamicObstaclePredictionNode::pose_callback(const nav_msgs::Odometry::ConstPtr& pose_msg){
  this->robot_pose=pose_msg->pose.pose;
}

void DynamicObstaclePredictionNode::speed_callback(const ackermann_msgs::AckermannDrive::ConstPtr& ackermann_msg){
  //Approximate the speed in the local axis X, using the steering angle as the angular angle.
  this->robot_speed=ackermann_msg->speed*cos(ackermann_msg->steering_angle);
}

void DynamicObstaclePredictionNode::callback(const sensor_msgs::PointCloud2::ConstPtr& ground_obstacles_msg,const sensor_msgs::PointCloud2::ConstPtr& ouster_obstacles_msg,const detection_msgs::PositionListConstPtr& obstacles_position_msg)
{
  if (ground_obstacles_msg == NULL) return;
  pcl::PCLPointCloud2 aux;
  PointCloud::Ptr ground_obstacles_pcl(new PointCloud);
  PointCloud::Ptr ouster_obstacles_pcl(new PointCloud);

  pcl_conversions::toPCL(*ground_obstacles_msg, aux);
  pcl::fromPCLPointCloud2(aux, *ground_obstacles_pcl);
  pcl_conversions::toPCL(*ouster_obstacles_msg, aux);
  pcl::fromPCLPointCloud2(aux, *ouster_obstacles_pcl);

  double now = ros::Time::now().toSec();
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
    geometry_msgs::Point global_position;
    // global_position.x = round((robot_pose.position.x + obstacles_position_msg->positions[i].x)*100.0)/100.0;
    // global_position.y = round((robot_pose.position.y + obstacles_position_msg->positions[i].y)*100.0)/100.0;
    global_position.x = robot_pose.position.x + obstacles_position_msg->positions[i].x;
    global_position.y = robot_pose.position.y + obstacles_position_msg->positions[i].y;
    //Apply rotation to get relative position
    geometry_msgs::Point relative_position;
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
  ground_cloud->header.frame_id = ground_obstacles_msg->header.frame_id;
  ground_cloud->header.stamp = ground_obstacles_msg->header.stamp.toNSec()/1e3;
  pc_obstXY_pub.publish (ground_cloud);

  // obstacles cloud
  ouster_cloud->is_dense = false;
  ground_cloud->width = (int) ouster_cloud->points.size();
  ouster_cloud->height = 1;
  ouster_cloud->header.frame_id = ground_obstacles_msg->header.frame_id;
  ouster_cloud->header.stamp = ground_cloud->header.stamp;
  pc_filtered_pub.publish (ouster_cloud);
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "DynamicObstaclePrediction");
  DynamicObstaclePredictionNode node;
  ros::spin();
  return 0;
}
