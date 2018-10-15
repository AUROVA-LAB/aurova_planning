#include "path_planning_demo_alg_node.h"

PathPlanningDemoAlgNode::PathPlanningDemoAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<PathPlanningDemoAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 10; //in [Hz]

  // [init publishers]
  this->marker_pub_ = this->public_node_handle_.advertise < visualization_msgs::MarkerArray > ("/visualization", 1);

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
  ////////////////////////////////////////////////////
  //debug
  int deb = this->alg_.readGraphFromFile(
      "/home/mice85/aurova-lab/aurova_ws/src/aurova_planning/path_planning_demo/src/zlinks.graph",
      "/home/mice85/aurova-lab/aurova_ws/src/aurova_planning/path_planning_demo/src/znodes.graph",
      "/home/mice85/aurova-lab/aurova_ws/src/aurova_planning/path_planning_demo/src/zgoals.graph");
  //ROS_INFO("num_links: %d", deb);
  //////////////////////////////////////////////////////

  // [fill msg structures]
  this->parseLinksToRosMarker(this->marker_array_);
  this->parseNodesToRosMarker(this->marker_array_);
  this->parseGoalsToRosMarker(this->marker_array_);

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->marker_pub_.publish(this->marker_array_);
  this->marker_array_.markers.clear();
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

int PathPlanningDemoAlgNode::parseLinksToRosMarker(visualization_msgs::MarkerArray& marker_array)
{
  int i, j;

  visualization_msgs::Marker marker;

  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "link";
  marker.id = 0;


  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = this->shape_ = visualization_msgs::Marker::SPHERE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  for (i = 1; i < this->alg_.planning_->st_links_[1].num_links; i++)
  {
    for (j = 0; j < this->alg_.planning_->st_links_[i].num_points; j++)
    {
      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = this->alg_.planning_->st_links_[i].points_x[j];
      marker.pose.position.y = this->alg_.planning_->st_links_[i].points_y[j];
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.id = marker.id + 1;
      marker_array.markers.push_back(marker);
    }
  }

  return 0;
}

int PathPlanningDemoAlgNode::parseNodesToRosMarker(visualization_msgs::MarkerArray& marker_array)
{
  int i;

  visualization_msgs::Marker marker;

  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "nodes";
  marker.id = 0;


  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = this->shape_ = visualization_msgs::Marker::SPHERE;

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

  for (i = 1; i < this->alg_.planning_->st_nodes_[1].num_nodes; i++)
  {
      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = this->alg_.planning_->st_nodes_[i].pose_x;
      marker.pose.position.y = this->alg_.planning_->st_nodes_[i].pose_y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.id = marker.id + 1;
      marker_array.markers.push_back(marker);
  }

  return 0;
}

int PathPlanningDemoAlgNode::parseGoalsToRosMarker(visualization_msgs::MarkerArray& marker_array)
{
  int i;

  visualization_msgs::Marker marker;

  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "goals";
  marker.id = 0;


  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = this->shape_ = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  for (i = 1; i < this->alg_.planning_->st_goals_[1].num_goals; i++)
  {
      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = this->alg_.planning_->st_goals_[i].pose_x;
      marker.pose.position.y = this->alg_.planning_->st_goals_[i].pose_y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.id = marker.id + 1;
      marker_array.markers.push_back(marker);
  }

  return 0;
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < PathPlanningDemoAlgNode > (argc, argv, "path_planning_demo_alg_node");
}
