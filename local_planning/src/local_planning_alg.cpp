#include "local_planning_alg.h"

LocalPlanningAlgorithm::LocalPlanningAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

LocalPlanningAlgorithm::~LocalPlanningAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void LocalPlanningAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// LocalPlanningAlgorithm Public API

void LocalPlanningAlgorithm::potentialForcesMap(pcl::PointCloud<pcl::PointXYZ> free_space, 
                                                int size, int offset, int scale,
                                                cv::Mat& pf_map)
{
  float in_out_flag;
  bool measure_dist = true;
  int i, j, k, u1, v1, u2, v2;
  std::vector<cv::Point2f> vertices(free_space.points.size() + 1);
  cv::Mat polygon(size, size, CV_8UC1, EMPTY_PIXEL);
  
  //build polygon from vertex
  for (i = 0; i < free_space.points.size()-1; ++i)
  {
    u1 = (int)(free_space.points[i].x * scale) + offset;
    v1 = (int)(free_space.points[i].y * scale) + offset;
    u2 = (int)(free_space.points[i+1].x * scale) + offset;
    v2 = (int)(free_space.points[i+1].y * scale) + offset;
    cv::line(polygon, cv::Point(u1, v1),  cv::Point(u2, v2), cv::Scalar(MAX_PIXEL), 1);
  }
  u1 = u2;
  v1 = v2;
  u2 = (int)(free_space.points[0].x * scale) + offset;
  v2 = (int)(free_space.points[0].y * scale) + offset;
  cv::line(polygon, cv::Point(u1, v1),  cv::Point(u2, v2), cv::Scalar(MAX_PIXEL), 1);
  
  // build contour
  vector<vector<cv::Point> > contour;
  cv::findContours(polygon, contour, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  
  // build potential map
  float wr = 1.0, wa = 1.0, ar = 0.5; //TODO: get drom param
  float force, distance, min_distance;
  cv::Mat forces_map(polygon.size(), CV_32F);
  for(i = 0; i < pf_map.rows; i++)
  {
    for(j = 0; j < pf_map.cols; j++)
    {
      in_out_flag = (float)cv::pointPolygonTest(contour[0], cv::Point2f((float)j, (float)i), measure_dist);
      if (in_out_flag > 0)
      {
        force = 0.0;
        min_distance = (float)size * 2;
        for (k = 0; k < free_space.points.size(); ++k)
        {
          u1 = (int)(free_space.points[k].x * scale) + offset;
          v1 = (int)(free_space.points[k].y * scale) + offset;
          
          distance = sqrt(pow((float)(u1 - j), 2.0) + pow((float)(v1 - i), 2.0));
          
          if (distance < min_distance)
          {
            min_distance = distance;
          }
        }
        if (min_distance == 0.0) min_distance = 1.0;
        force = wr / pow(min_distance, ar);
        forces_map.at<float>(i,j) = force;
      }
      else
      {
        forces_map.at<float>(i,j) = EMPTY_PIXEL;
      }
    }
  }
  
  // normalize map
  double min_val, max_val;
  cv::Point max_dist_pt;
  cv::Point min_dist_pt;
  cv::minMaxLoc(forces_map, &min_val, &max_val, &min_dist_pt, &max_dist_pt);
  ROS_INFO("max_val: %f, min_val: %f", max_val, min_val);
  max_val = max_val - min_val;
  for(i = 0; i < polygon.rows; i++)
  {
    for(j = 0; j < polygon.cols; j++)
    {
    
      if (forces_map.at<float>(i,j) != 0.0)
      {
        pf_map.at<cv::Vec3b>(i,j)[0] = (uchar)(((forces_map.at<float>(i,j) - min_val) / max_val) * MAX_PIXEL);
        pf_map.at<cv::Vec3b>(i,j)[1] = pf_map.at<cv::Vec3b>(i,j)[0];
        pf_map.at<cv::Vec3b>(i,j)[2] = pf_map.at<cv::Vec3b>(i,j)[0];
      }
    }
  }
  
  cv::applyColorMap(pf_map, pf_map, cv::COLORMAP_JET);
  //pf_map = polygon;
  //cv::imshow("Source", polygon);	
  return;
}
