#include "path_planning_demo_alg_node.h"

PathPlanningDemoAlgNode::PathPlanningDemoAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<PathPlanningDemoAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]

  // [init publishers]
  this->marker_pub_ = this->public_node_handle_.advertise < visualization_msgs::Marker > ("/visualization", 1);

  // [init subscribers]

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

PathPlanningDemoAlgNode::~PathPlanningDemoAlgNode(void)
{
  // [free dynamic memory]
}

void PathPlanningDemoAlgNode::mainNodeThread(void)
{
  static int end_visualization = 1;
  static int i = 1;
  static int j = 0;

  ////////////////////////////////////////////////////
  //debug
  int deb = this->alg_.readGraphFromFile(
      "/home/mice85/aurova-lab/aurova_ws/src/aurova_planning/path_planning_demo/src/zlinks.graph", "", "");
  //ROS_INFO("num_links: %d", deb);
  //////////////////////////////////////////////////////

  // [fill msg structures]
  if (i < this->alg_.planning_->st_links_[1].num_links)
  {
    this->parseLinksToRosMarker(this->marker_, this->alg_.planning_->st_links_[i].points_x[j],
                                this->alg_.planning_->st_links_[i].points_y[j]);
    j++;
    if (j >= this->alg_.planning_->st_links_[i].num_points)
    {
      j = 0;
      i++;
    }
  }else{
    end_visualization = 0;
  }

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  if (end_visualization)
    this->marker_pub_.publish(this->marker_);
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void PathPlanningDemoAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void PathPlanningDemoAlgNode::addNodeDiagnostics(void)
{
}

int PathPlanningDemoAlgNode::parseLinksToRosMarker(visualization_msgs::Marker& marker, float point_x, float point_y)
{
  static int id_shape = 0;

  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "link";
  marker.id = id_shape;
  id_shape++;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = this->shape_ = visualization_msgs::Marker::SPHERE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = point_x;  //this->alg_.planning_->st_links_[i].points_x[j];
  marker.pose.position.y = point_y;  //this->alg_.planning_->st_links_[i].points_y[j];
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  return 0;
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < PathPlanningDemoAlgNode > (argc, argv, "path_planning_demo_alg_node");
}
