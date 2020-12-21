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
                                                vector<vector<cv::Point> >& contour,
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
        
        /*
        u1 = (int)(goal_lidar.x * pf_config.scale) + pf_config.offset_x;
        v1 = (int)(goal_lidar.y * pf_config.scale) + pf_config.offset_y;
        distance = sqrt(pow((float)(u1 - j), 2.0) + pow((float)(v1 - i), 2.0));
        
        if (distance == 0.0) distance = 1.0;
        force = force - pf_config.wa / pow(distance, pf_config.aa);
        */
        
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
  pf_config.min_pt_x = (int)(goal_lidar.x * pf_config.scale) + pf_config.offset_x; //min_dist_pt.x;
  pf_config.min_pt_y = (int)(goal_lidar.y * pf_config.scale) + pf_config.offset_y; //min_dist_pt.y;
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

void LocalPlanningAlgorithm::findTransitableAreas(cv::Mat pf_map, 
                                                  vector<vector<cv::Point> > contour, 
                                                  cv::Point2f goal_lidar,
                                                  PFConfig pf_config, 
                                                  cv::Mat& roads_map)
{

  // sobel calculation
  cv::Mat pf_map_gray;
  cv::Mat grad;
  cv::Mat grad_x, grad_y;
  cv::Mat abs_grad_x, abs_grad_y;
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
  cv::cvtColor(pf_map, pf_map_gray, CV_BGR2GRAY);
  cv::Sobel(pf_map_gray, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
  cv::Sobel(pf_map_gray, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
  convertScaleAbs(grad_x, abs_grad_x);
  convertScaleAbs(grad_y, abs_grad_y);
  addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
  grad.copyTo(roads_map);
  cv::cvtColor(roads_map, roads_map, cv::COLOR_GRAY2BGR);
  
  //cv::threshold(roads_map, roads_map, 10, 255, cv::THRESH_BINARY_INV);
  
  int i, j, u, v;
  float in_out_flag = 0.0, distance, force;
  bool measure_dist = true;
  float threshold = (float)pf_config.threshold_grad;
  cv::Mat forces_map(roads_map.size(), CV_32F);
  for(i = 0; i < roads_map.rows; i++)
  {
    for(j = 0; j < roads_map.cols; j++)
    {
       in_out_flag = (float)cv::pointPolygonTest(contour[0], cv::Point2f((float)j, (float)i), measure_dist);
       if (in_out_flag > 0.0 && roads_map.at<cv::Vec3b>(i,j)[0] < pf_config.threshold_grad)
       {
       
         u = (int)(goal_lidar.x * pf_config.scale) + pf_config.offset_x;
         v = (int)(goal_lidar.y * pf_config.scale) + pf_config.offset_y;
         distance = sqrt(pow((float)(u - j), 2.0) + pow((float)(v - i), 2.0));
        
         if (distance == 0.0) distance = 1.0;
         force = 0.0; //pf_config.wa / pow(distance, pf_config.aa);
         force = force + abs((float)(roads_map.at<cv::Vec3b>(i,j)[0]) - threshold) / threshold;
         forces_map.at<float>(i,j) = force;
       }
       else
       {
         forces_map.at<float>(i,j) = 0.0;
       }
    }
  }
  
  // normalize map
  double min_val, max_val;
  cv::Point max_dist_pt;
  cv::Point min_dist_pt;
  cv::minMaxLoc(forces_map, &min_val, &max_val, &min_dist_pt, &max_dist_pt);
  for(i = 0; i < roads_map.rows; i++)
  {
    for(j = 0; j < roads_map.cols; j++)
    {
      roads_map.at<cv::Vec3b>(i,j)[0] = (uchar)((forces_map.at<float>(i,j) / max_val)  * MAX_PIXEL);
      roads_map.at<cv::Vec3b>(i,j)[1] = roads_map.at<cv::Vec3b>(i,j)[0];
      roads_map.at<cv::Vec3b>(i,j)[2] = roads_map.at<cv::Vec3b>(i,j)[0];
    }
  }
  return;
}

void LocalPlanningAlgorithm::findLocalGoal(pcl::PointCloud<pcl::PointXYZ> free_space, 
                                           cv::Point2f goal_lidar,
                                           PFConfig& pf_config,
                                           vector<vector<cv::Point> >& contour,
                                           int& radious,
                                           cv::Mat& plot_img,
                                           vector<cv::Point2d>& goal_candidates,
                                           cv::Point2d& local_goal)
{
  int ini = 0;
  int end = 360;
  int res = 1;
  float x, y, in_out_flag, angle, range, min_force;
  float force, min_distance, distance;
  bool measure_dist = true;
  int i, j, k, u1, v1, u2, v2;
  cv::Point2d point;
  cv::Mat polygon(pf_config.size_y, pf_config.size_x, CV_8UC1, EMPTY_PIXEL);
  
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
  polygon.copyTo(plot_img);
  cv::cvtColor(plot_img, plot_img, cv::COLOR_GRAY2BGR);
  
  // build contour
  cv::findContours(polygon, contour, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  
  //range = sqrt(pow(goal_lidar.x, 2) + pow(goal_lidar.y, 2)) / 2.0;
  range = pf_config.rad_max;
  radious = (int)(range * pf_config.scale);
  min_force = 1000000;
  for (i = ini; i < end; i += res)
  { 
    angle = (float)i;
    x = range * cos(angle);
    y = range * sin(angle);
    u1 = (int)(x * pf_config.scale) + pf_config.offset_x;
    v1 = (int)(y * pf_config.scale) + pf_config.offset_y;
    
    in_out_flag = (float)cv::pointPolygonTest(contour[0], cv::Point2f((float)u1, (float)v1), measure_dist);
    
    if (in_out_flag > 0)
    {
      min_distance = sqrt(pow((float)(pf_config.size_x), 2.0) + pow((float)(pf_config.size_y), 2.0));
      for (k = 0; k < free_space.points.size(); ++k)
      {
        u2 = (int)(free_space.points[k].x * pf_config.scale) + pf_config.offset_x;
        v2 = (int)(free_space.points[k].y * pf_config.scale) + pf_config.offset_y;
          
        distance = sqrt(pow((float)(u2 - u1), 2.0) + pow((float)(v2 - v1), 2.0));
          
        if (distance < min_distance)
        {
          min_distance = distance;
        }
      }
      
      force = min_distance;
      
      if (force > (2.0 * pf_config.scale)) //TODO: get from param
      {
        point.x = (float)u1;
        point.y = (float)v1;
        goal_candidates.push_back(point);
      
        u2 = (int)(goal_lidar.x * pf_config.scale) + pf_config.offset_x;
        v2 = (int)(goal_lidar.y * pf_config.scale) + pf_config.offset_y; 
      
        distance = sqrt(pow((float)(u2 - u1), 2.0) + pow((float)(v2 - v1), 2.0));
      
        if (distance < min_force)
        {
          min_force = distance;
          local_goal.x = (float)u1;
          local_goal.y = (float)v1;
        }
      }
    }
  }
  
  
  return;
}

void LocalPlanningAlgorithm::findControlAction (cv::Point2d local_goal,
                                                Pose2D base_pose,
                                                PFConfig pf_config,
                                                CtrlConfig ctrl_config,
                                                vector<vector<cv::Point> > contour,
                                                ackermann_msgs::AckermannDriveStamped& ackermann_state,
                                                cv::Mat& plot_img)
{
  float k_sp = (ctrl_config.v_max - ctrl_config.v_min) / ctrl_config.max_angle;
  float delta_t = ctrl_config.delta_time * 20; //TODO: from param
  float d_vehicle = ctrl_config.v_length;
  float pose_yaw_prev = base_pose.yaw;
  float pose_x_prev = base_pose.x;
  float pose_y_prev = base_pose.y;
  float cost, min_cost, dist_front, dist_rear;
  float min_st = 0.0;
  float min_sp = 0.0;
  cv::Point2d min_uv, min_uv_front;
  float base_front_x = base_pose.x + d_vehicle * cos(base_pose.yaw);
  float base_front_y = base_pose.y + d_vehicle * sin(base_pose.yaw);
  float k_diff = 0.0; //TODO: get from param
  float k_abs = 1.0; //TODO: get from param
  
  dist_rear = sqrt(pow(local_goal.x - base_pose.x, 2) + pow(local_goal.y - base_pose.y, 2));
	dist_front = sqrt(pow(local_goal.x - base_front_x, 2) + pow(local_goal.y - base_front_y, 2));
	min_cost = dist_rear * k_abs - (dist_front - dist_rear) * k_diff;
  for (float st = -1 * ctrl_config.max_angle; st < ctrl_config.max_angle; st += ctrl_config.delta_angle)
  {
    // FRONT
    float lineal_speed = ctrl_config.v_max - abs(st) * k_sp;
    float steering_radians = st * M_PI / 180.0;
    
    float angular_speed_yaw = (lineal_speed / d_vehicle) * sin(steering_radians);
    float pose_yaw = pose_yaw_prev + angular_speed_yaw * delta_t;
    
    float lineal_speed_x = lineal_speed * cos(pose_yaw) * cos(steering_radians);
    float lineal_speed_y = lineal_speed * sin(pose_yaw) * cos(steering_radians);
    float pose_x = pose_x_prev + lineal_speed_x * delta_t;
    float pose_y = pose_y_prev + lineal_speed_y * delta_t;
    float pose_front_x = pose_x + d_vehicle * cos(pose_yaw);
    float pose_front_y = pose_y + d_vehicle * sin(pose_yaw);
    
    cv::Point2d uv, uv_front;
		float measure_dist;
		uv.x = (float)(pose_x * pf_config.scale) + pf_config.offset_x;
		uv.y = (float)(pose_y * pf_config.scale) + pf_config.offset_y;
		float in_out_flag = (float)cv::pointPolygonTest(contour[0], uv, measure_dist);
		if (in_out_flag > 0)
    {
    	uv.x = (int)(pose_x * pf_config.scale) + pf_config.offset_x;
			uv.y = (int)(pose_y * pf_config.scale) + pf_config.offset_y;
			cv::circle(plot_img, uv, 0, CV_RGB(MAX_PIXEL, MAX_PIXEL, EMPTY_PIXEL), -1);
			uv_front.x = (int)(pose_front_x * pf_config.scale) + pf_config.offset_x;
			uv_front.y = (int)(pose_front_y * pf_config.scale) + pf_config.offset_y;
			cv::circle(plot_img, uv_front, 0, CV_RGB(EMPTY_PIXEL, MAX_PIXEL, MAX_PIXEL), -1);
			
			dist_rear = sqrt(pow(local_goal.x - uv.x, 2) + pow(local_goal.y - uv.y, 2));
			dist_front = sqrt(pow(local_goal.x - uv_front.x, 2) + pow(local_goal.y - uv_front.y, 2));
			cost = dist_rear * k_abs - (dist_front - dist_rear) * k_diff;
			
			if (cost < min_cost)
			{
			  min_uv.x = (int)(pose_x * pf_config.scale) + pf_config.offset_x;
				min_uv.y = (int)(pose_y * pf_config.scale) + pf_config.offset_y;
				min_uv_front.x = (int)(pose_front_x * pf_config.scale) + pf_config.offset_x;
				min_uv_front.y = (int)(pose_front_y * pf_config.scale) + pf_config.offset_y;
				
				min_cost = cost;
				min_st = (st*PI)/180.0;
		    min_sp = lineal_speed;
			}
    }
		
		// REAR
		lineal_speed = -1 * lineal_speed;
    
    angular_speed_yaw = (lineal_speed / d_vehicle) * sin(steering_radians);
    pose_yaw = pose_yaw_prev + angular_speed_yaw * delta_t;
    
    lineal_speed_x = lineal_speed * cos(pose_yaw) * cos(steering_radians);
    lineal_speed_y = lineal_speed * sin(pose_yaw) * cos(steering_radians);
    pose_x = pose_x_prev + lineal_speed_x * delta_t;
    pose_y = pose_y_prev + lineal_speed_y * delta_t;
    pose_front_x = pose_x + d_vehicle * cos(pose_yaw);
    pose_front_y = pose_y + d_vehicle * sin(pose_yaw);
    
		measure_dist;
		uv.x = (float)(pose_x * pf_config.scale) + pf_config.offset_x;
		uv.y = (float)(pose_y * pf_config.scale) + pf_config.offset_y;
		in_out_flag = (float)cv::pointPolygonTest(contour[0], uv, measure_dist);
		if (in_out_flag > 0)
    {
    	uv.x = (int)(pose_x * pf_config.scale) + pf_config.offset_x;
			uv.y = (int)(pose_y * pf_config.scale) + pf_config.offset_y;
			cv::circle(plot_img, uv, 0, CV_RGB(MAX_PIXEL, MAX_PIXEL, EMPTY_PIXEL), -1);
			uv_front.x = (int)(pose_front_x * pf_config.scale) + pf_config.offset_x;
			uv_front.y = (int)(pose_front_y * pf_config.scale) + pf_config.offset_y;
			cv::circle(plot_img, uv_front, 0, CV_RGB(EMPTY_PIXEL, MAX_PIXEL, MAX_PIXEL), -1);
			
			dist_rear = sqrt(pow(local_goal.x - uv.x, 2) + pow(local_goal.y - uv.y, 2));
			dist_front = sqrt(pow(local_goal.x - uv_front.x, 2) + pow(local_goal.y - uv_front.y, 2));
			cost = dist_rear * k_abs - (dist_front - dist_rear) * k_diff;
			
			if (cost < min_cost)
			{
			  min_uv.x = (int)(pose_x * pf_config.scale) + pf_config.offset_x;
				min_uv.y = (int)(pose_y * pf_config.scale) + pf_config.offset_y;
				min_uv_front.x = (int)(pose_front_x * pf_config.scale) + pf_config.offset_x;
				min_uv_front.y = (int)(pose_front_y * pf_config.scale) + pf_config.offset_y;
				
				min_cost = cost;
				min_st = (st*PI)/180.0;
		    min_sp = lineal_speed;
			}
    }
  }
  
  ackermann_state.drive.steering_angle = min_st;
	ackermann_state.drive.speed = min_sp;
	cv::circle(plot_img, min_uv, 1, CV_RGB(MAX_PIXEL, EMPTY_PIXEL, EMPTY_PIXEL), -1);
	cv::circle(plot_img, min_uv_front, 1, CV_RGB(MAX_PIXEL, EMPTY_PIXEL, EMPTY_PIXEL), -1);
  
  return;
}
