#include <obstacle_detection.h>
#include <vector>
#include <complex> 
#include <filesystem>
#include <sstream>
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <omp.h>


using namespace std;
vector<vector<float>> controlActionCalculation(float speed, float steering, string filename);
Eigen::MatrixXf get_conv_filter(Eigen::MatrixXf& gaussian_kernel, Eigen::MatrixXf& safety_mat, int i, int j,
                                int& row_start, int& row_end, int& col_start, int& col_end);