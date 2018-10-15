#include "get_pose_from_tf_alg_node.h"

GetPoseFromTfAlgNode::GetPoseFromTfAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<GetPoseFromTfAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]

  // [init publishers]
  this->odometry_publisher_ = this->public_node_handle_.advertise < nav_msgs::Odometry > ("/odometry_filtered", 1);

  // [init subscribers]

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

GetPoseFromTfAlgNode::~GetPoseFromTfAlgNode(void)
{
  // [free dynamic memory]
}

void GetPoseFromTfAlgNode::mainNodeThread(void)
{
  std::string frame_id;
  std::string child_id;

  // [read parameters]
  this->public_node_handle_.getParam("/frame_id_tf", frame_id);
  this->public_node_handle_.getParam("/child_id_tf", child_id);

  // [listen transform]
  try
  {
    this->listener_.lookupTransform(frame_id, child_id, ros::Time(0), this->transform_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  // [fill msg structures]
  /////////////////////////////////////////////////
  //// GENERATE MESSAGE
  // Header
  this->odometry_filtered_.header.stamp = ros::Time::now();
  this->odometry_filtered_.header.frame_id = frame_id;
  this->odometry_filtered_.child_frame_id = child_id;
  // Twist
  this->odometry_filtered_.twist.twist.linear.x = 0.0;
  this->odometry_filtered_.twist.twist.linear.y = 0.0;
  this->odometry_filtered_.twist.twist.linear.z = 0.0;
  this->odometry_filtered_.twist.twist.angular.x = 0.0;
  this->odometry_filtered_.twist.twist.angular.y = 0.0;
  this->odometry_filtered_.twist.twist.angular.z = 0.0;
  // Pose
  this->odometry_filtered_.pose.pose.position.x = this->transform_.getOrigin().x();
  this->odometry_filtered_.pose.pose.position.y = this->transform_.getOrigin().y();
  this->odometry_filtered_.pose.pose.position.z = this->transform_.getOrigin().z();
  this->odometry_filtered_.pose.pose.orientation.x = this->transform_.getRotation().x();
  this->odometry_filtered_.pose.pose.orientation.y = this->transform_.getRotation().y();
  this->odometry_filtered_.pose.pose.orientation.z = this->transform_.getRotation().z();
  this->odometry_filtered_.pose.pose.orientation.w = this->transform_.getRotation().w();
  /////////////////////////////////////////////////

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->odometry_publisher_.publish(this->odometry_filtered_);

}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void GetPoseFromTfAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void GetPoseFromTfAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < GetPoseFromTfAlgNode > (argc, argv, "get_pose_from_tf_alg_node");
}
