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
                                                int size, int offset,
                                                cv::Mat& pf_map)
{
  float in_out_flag;
  bool measure_dist = true;
  int i, j, u1, v1, u2, v2;
  std::vector<cv::Point2f> vertices(free_space.points.size() + 1);
  cv::Mat polygon(size, size, CV_8UC1, EMPTY_PIXEL);
  
  //build polygon from vertex
  for (i = 0; i < free_space.points.size()-1; ++i)
  {
    u1 = (int)(free_space.points[i].x) + offset;
    v1 = (int)(free_space.points[i].y) + offset;
    u2 = (int)(free_space.points[i+1].x) + offset;
    v2 = (int)(free_space.points[i+1].y) + offset;
    cv::line(polygon, cv::Point(u1, v1),  cv::Point(u2, v2), cv::Scalar(MAX_PIXEL), 1);
  }
  u1 = u2;
  v1 = v2;
  u2 = (int)(free_space.points[0].x) + offset;
  v2 = (int)(free_space.points[0].y) + offset;
  cv::line(polygon, cv::Point(u1, v1),  cv::Point(u2, v2), cv::Scalar(MAX_PIXEL), 1);
  
  // build contour
  vector<vector<cv::Point> > contour;
  cv::findContours(polygon, contour, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  
  // build potential map
  cv::Mat raw_dist(polygon.size(), CV_32F);
  for(i = 0; i < pf_map.rows; i++)
  {
    for(j = 0; j < pf_map.cols; j++)
    {
      in_out_flag = (float)cv::pointPolygonTest(contour[0], cv::Point2f((float)j, (float)i), measure_dist);
      raw_dist.at<float>(i, j) = in_out_flag;
    }
  }
  
  // normalize map
  double min_val, max_val;
  cv::Point max_dist_pt;
  cv::Point min_dist_pt;
  cv::minMaxLoc(raw_dist, &min_val, &max_val, &min_dist_pt, &max_dist_pt);
  min_val = abs(min_val);
  max_val = abs(max_val);
  cv::Mat drawing = cv::Mat::zeros(polygon.size(), CV_8UC3);
  for(i = 0; i < polygon.rows; i++)
  {
    for(j = 0; j < polygon.cols; j++)
    {
      if(raw_dist.at<float>(i,j) < 0)
      {
        drawing.at<cv::Vec3b>(i,j)[0] = (uchar)(255 - abs(raw_dist.at<float>(i,j)) * 255 / min_val);
      }
      else if( raw_dist.at<float>(i,j) > 0 )
      {
        drawing.at<cv::Vec3b>(i,j)[2] = (uchar)(255 - raw_dist.at<float>(i,j) * 255 / max_val);
      }
      else
      {
        drawing.at<cv::Vec3b>(i,j)[0] = 255;
        drawing.at<cv::Vec3b>(i,j)[1] = 255;
        drawing.at<cv::Vec3b>(i,j)[2] = 255;
      }
    }
  }
  
  
  
  
  pf_map = drawing;
  //in_out_flag = cv::pointPolygonTest(contour, pt, measure_dist);
  //cv::imshow("Source", polygon);	
  return;
}
