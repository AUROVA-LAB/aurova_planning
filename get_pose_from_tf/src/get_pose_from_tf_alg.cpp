#include "get_pose_from_tf_alg.h"

GetPoseFromTfAlgorithm::GetPoseFromTfAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

GetPoseFromTfAlgorithm::~GetPoseFromTfAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void GetPoseFromTfAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// GetPoseFromTfAlgorithm Public API
