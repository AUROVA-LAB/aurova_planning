#include "get_pose_from_tf_alg_node.h"

GetPoseFromTfAlgNode::GetPoseFromTfAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<GetPoseFromTfAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  
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
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void GetPoseFromTfAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void GetPoseFromTfAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<GetPoseFromTfAlgNode>(argc, argv, "get_pose_from_tf_alg_node");
}
