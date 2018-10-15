#include "path_planning_demo_alg.h"

PathPlanningDemoAlgorithm::PathPlanningDemoAlgorithm(void)
{
  this->planning_ = new TopologicPlanning();

  pthread_mutex_init(&this->access_, NULL);
}

PathPlanningDemoAlgorithm::~PathPlanningDemoAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void PathPlanningDemoAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_ = config;

  this->unlock();
}

//////////////////////////////////////////////////////////////
// PathPlanningDemoAlgorithm Public API
//////////////////////////////////////////////////////////////

int PathPlanningDemoAlgorithm::readGraphFromFile(char *pathLinks, char *pathNodes, char *pathGoals)
{
  this->planning_->loadLinksFromFile(pathLinks);

  return 0;
}
