#include "local_planning_alg.h"

LocalPlanningAlgorithm::LocalPlanningAlgorithm(void)
{
  pthread_mutex_init(&this->access_, NULL);
}

LocalPlanningAlgorithm::~LocalPlanningAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void LocalPlanningAlgorithm::config_update(Config &config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_ = config;

  this->unlock();
}

// LocalPlanningAlgorithm Public API
