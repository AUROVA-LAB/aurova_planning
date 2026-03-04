#include "secure_actions.h"

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Global variables
float max_angle = 24.0;
float delta_angle = 2.0;
float v_length = 1.05;
float delta_arc = 1.0;
float max_arc = 2.0;
float v_max = 1.3;
float margin = 1.0;
float base_in_lidarf_x = 0.55;
float base_in_lidarf_y = 0.0;
float base_in_lidarf_yaw = 0.0;

vector<vector<float>> controlActionCalculation(float speed, float steering, string filename)
{
    PointCloud::Ptr input_cloud = get_pc_from_image(filename);
    PointCloud::Ptr obstacles_cloud = detect_obstacles(input_cloud);
    pcl::PointXYZ point;
    vector<vector<float>> safety_matrix = vector<vector<float>>(int(max_angle * 2 / delta_angle) + 1, vector<float>(9, 0.0));
    speed = speed / v_max; // Normalize speed, steering not needed
    float min_match_distance = 100;
    float min_match_speed = 0;
    float min_match_steering = 0;
    //////////////////////////////////////////////////////////////////////////////////////////
    // CHECK POSIBLE CONTROL ACTIONS
    for (int i = 0; -max_angle + delta_angle * i <= max_angle; i++)
    {
        float st = -max_angle + delta_angle * i;
        //// CHECK REAR-FRONT ACTIONS
        for (int j = 0; j < 9; j++)
        {
            float dir = 0.25 * j - 1;
            float steering_radians = st * M_PI / 180.0;
            bool flag_collision_risk = false;
            float dist = norm(complex(speed - dir, st - steering));
            if (dist < min_match_distance)
            {
                min_match_distance = dist;
                min_match_speed = j;
                min_match_steering = i;
            }
            float w_current, r;
            if (abs(steering_radians) >= 0.001)
            {
                r = v_length / tan(steering_radians);
                w_current = base_in_lidarf_yaw + dir / r;
            }
            else
                w_current = base_in_lidarf_yaw;

            //// CALCULATE TRAJECTORY ARC FROM STEERING
            for (float arc = delta_arc;
                 arc <= max_arc; arc += delta_arc)
            {
                float x_current = base_in_lidarf_x + arc * dir * cos(w_current*arc);
                float y_current = base_in_lidarf_y + arc * dir * sin(w_current*arc);
                point.x = x_current;
                point.y = y_current;
                
                //// CHECK IF COLLISION RISK
                for (int i = 0; i < obstacles_cloud->points.size(); i++)
                {
                    if (obstacles_cloud->points[i].x < point.x + margin && obstacles_cloud->points[i].x > point.x - margin && obstacles_cloud->points[i].y < point.y + margin && obstacles_cloud->points[i].y > point.y - margin)
                    {
                        flag_collision_risk = true;
                        break;
                    }
                }
            }

            safety_matrix[i][j] = flag_collision_risk ? 1.0 : 0.5;
        }
    }
    safety_matrix[min_match_steering][min_match_speed] = 0.0;
    return safety_matrix;
}

Eigen::MatrixXf get_conv_filter(Eigen::MatrixXf& gaussian_kernel, Eigen::MatrixXf& safety_mat, int i, int j,
                                int& row_start, int& row_end, int& col_start, int& col_end){
    Eigen::MatrixXf aux_kernel;
    int kernel_row_start, kernel_col_start;
    //Get row limits
    if(i-floor(gaussian_kernel.rows()/2)<0){
        row_start=0; row_end=i+floor(gaussian_kernel.rows()/2)+1;
        kernel_row_start=floor(gaussian_kernel.rows()/2)-i;
    }
    else if(i+floor(gaussian_kernel.rows()/2)>=safety_mat.rows()){
        row_start=i-floor(gaussian_kernel.rows()/2); row_end=safety_mat.rows();
        kernel_row_start=0;
    }
    else{
        row_start=i-floor(gaussian_kernel.rows()/2); row_end=i+floor(gaussian_kernel.rows()/2)+1;
        kernel_row_start=0;
    }
    //Get column limits
    if(j-floor(gaussian_kernel.cols()/2)<0){
        col_start=0; col_end=j+floor(gaussian_kernel.cols()/2)+1;
        kernel_col_start=floor(gaussian_kernel.cols()/2)-j;
    }
    else if(j+floor(gaussian_kernel.cols()/2)>=safety_mat.cols()){
        col_start=j-floor(gaussian_kernel.cols()/2); col_end=safety_mat.cols();
        kernel_col_start=0;
    }
    else{
        col_start=j-floor(gaussian_kernel.cols()/2); col_end=j+floor(gaussian_kernel.cols()/2)+1;
        kernel_col_start=0;
    }
    
    aux_kernel = gaussian_kernel.block(kernel_row_start,kernel_col_start,row_end-row_start, col_end-col_start); 
    aux_kernel /= aux_kernel.sum();
    return aux_kernel;
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        cout << "Usage: " << argv[0] << " path/to/dataset/" << endl;
        return -1;
    }
    string dataset_path = argv[1];

    // Precomputed 3x3 Gaussian kernel (normalized)
    Eigen::MatrixXf gaussian_kernel(3,5);
    gaussian_kernel << 1, 2, 4, 2, 1, 
        2, 4, 24, 4, 2,
        1, 2, 4, 2, 1;
    // Read "train" and "val" folders inside dataset_path
    vector<string> splits = {"train/", "val/"};
    vector<filesystem::directory_entry> entries;
    for (const auto &split : splits)
    {
        string path = dataset_path + split;
        
        for (const auto &entry : filesystem::directory_iterator(path)){ entries.push_back(entry);}
        
    }
    #pragma omp parallel for schedule(dynamic)
    for (size_t id = 0; id < entries.size(); id++)
    {
        string lidar_folder = string(entries[id].path()) + "/lidar";
        string measurements_filename = string(entries[id].path()) + "/commands.csv";
        string output_filename = string(entries[id].path()) + "/safety_matrices.txt";
        int lidar_index = 1;
        if (filesystem::exists(output_filename))
        {
            continue; // Skip if output file already exists
        }
        // Open measurements.txt
        ifstream measurements_file(measurements_filename);
        ofstream output_file(output_filename);
        if (!measurements_file.is_open())
        {
            #pragma omp critical
            {
                cerr << "Could not open file: " << measurements_filename << endl; 
            }
            continue;
        }
        #pragma omp critical
        {
            cout << "Processing: " << entries[id].path() << endl;
        }
        // Example: read all lines (customize as needed)
        vector<string> measurements;
        string line;
        getline(measurements_file, line); // Skip header
        while (getline(measurements_file, line))
        {
            stringstream ss(line);
            string item;
            int id = 0;
            float speed, steering;
            while (getline(ss, item, ','))
            {
                if (id == 0)
                {
                    speed = stof(item);
                }
                else if (id == 1)
                {
                    steering = -stof(item); // The Y axis of Unreal Engine is inverted compared to ROS
                    break;
                }
                id++;
            }
            vector<vector<float>> safety_matrix = controlActionCalculation(speed, steering, lidar_folder + "/" + to_string(lidar_index) + ".png");
            lidar_index++;
            // Process safety_matrix as neede
            // Convert safety_matrix (vector<vector<float>>) to Eigen::MatrixXf
            Eigen::MatrixXf safety_mat(safety_matrix.size(), safety_matrix[0].size());
            for (size_t i = 0; i < safety_matrix.size(); ++i)
                for (size_t j = 0; j < safety_matrix[0].size(); ++j)
                    safety_mat(i, j) = safety_matrix[i][j];
            // Apply Gaussian blur (valid for small matrices, border pixels remain unchanged)
            Eigen::MatrixXf blurred=safety_mat;
            
            for (int i = 0; i < safety_mat.rows(); ++i)
            {
                for (int j = 0; j < safety_mat.cols(); ++j)
                {
                    int row_start, row_end, col_start, col_end;
                    Eigen::MatrixXf aux_kernel=get_conv_filter(gaussian_kernel, safety_mat, i, j, row_start, row_end, col_start, col_end);
                    blurred(i, j) =
                        (safety_mat.block(row_start, col_start, row_end-row_start, col_end-col_start).cwiseProduct(aux_kernel)).sum();
                    output_file << blurred(i, j)<< ",";
                }
            }
            // Move output_file write position back to overwrite the last character (comma)
            output_file.seekp(-1, ios_base::cur);
            output_file << endl;
        }
        measurements_file.close();
        output_file.close();
    }
    return 0;
}
