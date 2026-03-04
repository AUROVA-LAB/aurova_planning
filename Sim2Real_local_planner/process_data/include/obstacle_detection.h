#define PCL_NO_PRECOMPILE
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/pcl_macros.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include <iostream>
#include <math.h>

#include <limits>
#include <chrono> 
#include <string>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloud::Ptr get_pc_from_image(const std::string filename);
PointCloud::Ptr detect_obstacles(PointCloud::Ptr input_cloud, float minlen=0.5, float maxlen=8.0, float max_z=0.1, float voxel=0.15,
                                float smNorm=0.4, float laNorm=0.56, float threshold=0.05, int neighbours=10, float th_meanK=1.0);