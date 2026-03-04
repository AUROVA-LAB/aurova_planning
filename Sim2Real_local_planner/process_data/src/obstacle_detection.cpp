#include <obstacle_detection.h>


using namespace Eigen;
using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloud::Ptr get_pc_from_image(const string filename)
{

  cv::Mat img_range  = cv::imread(filename, cv::IMREAD_UNCHANGED);

  Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> depth_data , data_metrics;// matrix with image values and matrix qith image values into real range data
  cv2eigen(img_range,depth_data);       // convert img_range into eigen matrix
  float max_range=50.0;
  data_metrics = -depth_data*(max_range/65535.0); // resolution 16 bits -> 4mm. 
  data_metrics.array() += max_range; // convert to real range data

  
 
  PointCloud::Ptr point_cloud (new PointCloud);
  PointCloud::Ptr cloud_out (new PointCloud);

  point_cloud->width = img_range.cols; 
  point_cloud->height = img_range.rows;
  point_cloud->is_dense = false;
  point_cloud->points.resize (point_cloud->width * point_cloud->height);
  uint num_pix = 0;
  for (int i = 0;i<img_range.rows; i++){
      for (int j = 0;j<img_range.cols; j++){

        if (data_metrics(i,j)==0 or data_metrics(i,j)>=max_range)
          continue;

        float ang_h = 22.5 - (45.0/128.0)*i;
        ang_h = ang_h*M_PI/180.0;
        float ang_w = 184.0 - (360.0/2048.0)*j;
        ang_w = ang_w*M_PI/180.0;

        float z = data_metrics(i,j) * sin(ang_h);
        float y = sqrt(pow(data_metrics(i,j),2)-pow(z,2))*sin(ang_w);
        float x = sqrt(pow(data_metrics(i,j),2)-pow(z,2))*cos(ang_w);
        //asignacion de valores maximo y minimos

        point_cloud->points[num_pix].x = x;
        point_cloud->points[num_pix].y = y;
        point_cloud->points[num_pix].z = z;
        cloud_out->push_back(point_cloud->points[num_pix]); 
        num_pix++; 

      }
  } 
  
  cloud_out->is_dense = false;
  cloud_out->width = (int) cloud_out->points.size();
  cloud_out->height = 1;
  
  return cloud_out;
}

PointCloud::Ptr detect_obstacles(PointCloud::Ptr input_cloud, float minlen, float maxlen, float max_z, float voxel,
                                float smNorm, float laNorm, float threshold, int neighbours, float th_meanK)
{

    PointCloud::Ptr cloud (new PointCloud);

    for (int i = 0; i < (int) input_cloud->points.size(); i++)
    {
        double distance = sqrt(pow(input_cloud->points[i].x,2)+pow(input_cloud->points[i].y,2)+pow(input_cloud->points[i].z,2));
        if(distance<minlen || distance>maxlen || input_cloud->points[i].z> max_z)
        continue;
        cloud->push_back(input_cloud->points[i]);          
    }

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(voxel, voxel, voxel);
    vg.filter(*cloud);

    // crea KDTree para datos no organizados
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    if (cloud->isOrganized ())
        tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
    else
        tree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
    tree->setInputCloud (cloud);

    // Calcular las normales
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree);
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);
    ne.setRadiusSearch (smNorm);// Radio de búsqueda para vecinos cercanos
    ne.compute (*normals_small_scale);
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);
    ne.setRadiusSearch (laNorm);
    ne.compute (*normals_large_scale);

    // Create output cloud for DoN results
    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);
    copyPointCloud (*cloud, *doncloud);

    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
    don.setInputCloud (cloud);
    don.setNormalScaleLarge (normals_large_scale);
    don.setNormalScaleSmall (normals_small_scale);

    if (!don.initCompute ())
    {
        std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
        std::cerr << "Error:en deteccion de obstaculos " << std::endl;
        exit (EXIT_FAILURE);
    }
    // Compute DoN
    don.computeFeature (*doncloud);

    // Build the condition for filtering
    pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond ( new pcl::ConditionOr<pcl::PointNormal> ()  );
    range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (
                                new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
                                );

    // Build the filter
    pcl::ConditionalRemoval<pcl::PointNormal> condrem; 
    condrem.setCondition (range_cond);
    condrem.setInputCloud (doncloud);

    // Apply filter
    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);
    condrem.filter (*doncloud_filtered);
    doncloud = doncloud_filtered;

    // apply outlier filter

    pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
    sor.setInputCloud (doncloud);
    sor.setMeanK (neighbours);
    sor.setStddevMulThresh (th_meanK);
    sor.filter (*doncloud);

    // pointcloud pcl_obstacle 
    PointCloud::Ptr pcl_obstacle (new PointCloud);
    pcl_obstacle->is_dense = false;
    pcl_obstacle->width = cloud->width;
    pcl_obstacle->height = cloud->height;
    pcl_obstacle->points.resize (doncloud->width * doncloud->height);

    for (int i = 0; i < (int) doncloud->points.size(); i++)
    {
        pcl_obstacle->points[i].x= doncloud->points[i].x;     
        pcl_obstacle->points[i].y= doncloud->points[i].y; 
        pcl_obstacle->points[i].z= doncloud->points[i].z;   
    }

    return pcl_obstacle;
}