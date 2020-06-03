#include "global_planning_alg.h"

GlobalPlanningAlgorithm::GlobalPlanningAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

GlobalPlanningAlgorithm::~GlobalPlanningAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void GlobalPlanningAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// GlobalPlanningAlgorithm Public API
