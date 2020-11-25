#include "local_planning_alg_node.h"

LocalPlanningAlgNode::LocalPlanningAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<LocalPlanningAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 20;//in [Hz]
  
  this->local_planning_ = new LocalPlanning();

  // [init publishers]
  
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
