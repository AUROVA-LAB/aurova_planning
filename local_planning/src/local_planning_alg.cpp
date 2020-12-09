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
                                                cv::Point2f goal_lidar,
                                                PFConfig& pf_config,
                                                cv::Mat& pf_map)
{
  float in_out_flag;
  bool measure_dist = true;
  int i, j, k, u1, v1, u2, v2;
  cv::Mat polygon(pf_config.size_y, pf_config.size_x, CV_8UC1, EMPTY_PIXEL);
  //std::vector<cv::Point2f> vertices(free_space.points.size() + 1);
  
  //build polygon from vertex
  for (i = 0; i < free_space.points.size()-1; ++i)
  {
    u1 = (int)(free_space.points[i].x * pf_config.scale) + pf_config.offset_x;
    v1 = (int)(free_space.points[i].y * pf_config.scale) + pf_config.offset_y;
    u2 = (int)(free_space.points[i+1].x * pf_config.scale) + pf_config.offset_x;
    v2 = (int)(free_space.points[i+1].y * pf_config.scale) + pf_config.offset_y;
    cv::line(polygon, cv::Point(u1, v1),  cv::Point(u2, v2), cv::Scalar(MAX_PIXEL), 1);
  }
  u1 = u2;
  v1 = v2;
  u2 = (int)(free_space.points[0].x * pf_config.scale) + pf_config.offset_x;
  v2 = (int)(free_space.points[0].y * pf_config.scale) + pf_config.offset_y;
  cv::line(polygon, cv::Point(u1, v1),  cv::Point(u2, v2), cv::Scalar(MAX_PIXEL), 1);
  
  // build contour
  vector<vector<cv::Point> > contour;
  cv::findContours(polygon, contour, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  // build potential map
  float force, distance, min_distance;
  cv::Mat forces_map(polygon.size(), CV_32F);
  for(i = 0; i < pf_map.rows; i++)
  {
    for(j = 0; j < pf_map.cols; j++)
    {
      in_out_flag = (float)cv::pointPolygonTest(contour[0], cv::Point2f((float)j, (float)i), measure_dist);
      if (in_out_flag > 0)
      {
        force = 10.0;
        min_distance = sqrt(pow((float)(pf_config.size_x), 2.0) + pow((float)(pf_config.size_y), 2.0));
        for (k = 0; k < free_space.points.size(); ++k)
        {
          u1 = (int)(free_space.points[k].x * pf_config.scale) + pf_config.offset_x;
          v1 = (int)(free_space.points[k].y * pf_config.scale) + pf_config.offset_y;
          
          distance = sqrt(pow((float)(u1 - j), 2.0) + pow((float)(v1 - i), 2.0));
          
          if (distance < min_distance)
          {
            min_distance = distance;
          }
        }
        
        if (min_distance == 0.0) min_distance = 1.0;
        force = pf_config.wr / pow(min_distance, pf_config.ar);
        
        u1 = (int)(goal_lidar.x * pf_config.scale) + pf_config.offset_x;
        v1 = (int)(goal_lidar.y * pf_config.scale) + pf_config.offset_y;
        distance = sqrt(pow((float)(u1 - j), 2.0) + pow((float)(v1 - i), 2.0));
        
        if (distance == 0.0) distance = 1.0;
        force = force - pf_config.wa / pow(distance, pf_config.aa);
        
        forces_map.at<float>(i,j) = force;
      }
      else
      {
        forces_map.at<float>(i,j) = INVALID_PIXEL;
      }
    }
  }
  
  // normalize map
  double min_val, max_val;
  cv::Point max_dist_pt;
  cv::Point min_dist_pt;
  cv::minMaxLoc(forces_map, &min_val, &max_val, &min_dist_pt, &max_dist_pt);
  ROS_INFO("max_val: %f, min_val: %f", max_val, min_val);
  pf_config.min_pt_x = min_dist_pt.x;
  pf_config.min_pt_y = min_dist_pt.y;
  max_val = max_val - min_val;
  for(i = 0; i < polygon.rows; i++)
  {
    for(j = 0; j < polygon.cols; j++)
    {
    
      if (forces_map.at<float>(i,j) != INVALID_PIXEL)
      {
        pf_map.at<cv::Vec3b>(i,j)[0] = (uchar)(((forces_map.at<float>(i,j) - min_val) / max_val) * MAX_PIXEL);
        pf_map.at<cv::Vec3b>(i,j)[1] = pf_map.at<cv::Vec3b>(i,j)[0];
        pf_map.at<cv::Vec3b>(i,j)[2] = pf_map.at<cv::Vec3b>(i,j)[0];
      }
      else
      {
        pf_map.at<cv::Vec3b>(i,j)[0] = (uchar)(MAX_PIXEL);
        pf_map.at<cv::Vec3b>(i,j)[1] = pf_map.at<cv::Vec3b>(i,j)[0];
        pf_map.at<cv::Vec3b>(i,j)[2] = pf_map.at<cv::Vec3b>(i,j)[0];
      }
    }
  }
  
  //pf_map = polygon;
  //cv::imshow("Source", polygon);	
  return;
}

void LocalPlanningAlgorithm::findLocalMin(cv::Mat pf_map, cv::Mat& local_min_map)
{
  int i, j, n, m;
  bool is_min;
  int mask = 2;
  
  for(i = mask; i < local_min_map.rows-mask; i++)
  {
    for(j = mask; j < local_min_map.cols-mask; j++)
    {
      is_min = true;
      for(n = -mask; n <= mask; n++)
      {
        for(m = -mask; m <= mask; m++)
        {
          if (n != 0 && m != 0)
          {
            if (pf_map.at<cv::Vec3b>(i+n,j+m)[0] <= pf_map.at<cv::Vec3b>(i,j)[0])
            {
              is_min = false;
            }
          }
        }
      }
      if (is_min)
      {
        local_min_map.at<cv::Vec3b>(i,j)[0] = (uchar)(MAX_PIXEL);
        local_min_map.at<cv::Vec3b>(i,j)[1] = local_min_map.at<cv::Vec3b>(i,j)[0];
        local_min_map.at<cv::Vec3b>(i,j)[2] = local_min_map.at<cv::Vec3b>(i,j)[0];
      }
    }
  }
  
  return;
}
