
SAVE_FILE = 0;

% get entire trajectory
trajectory_x  = csvread('inputs/campus_poli_x.csv');
trajectory_y  = csvread('inputs/campus_poli_y.csv');
output_filename_nodes = 'outputs/znodes_campus_poli.graph';
output_filename_links    = 'outputs/zlinks_campus_poli.graph';
output_filename_goals  = 'outputs/zgoals_campus_poli.graph';

%--------------------------------------------------------------------------------------------------------------------
% define the nodes of grah-trajectory here
%--------------------------------------------------------------------------------------------------------------------
num_nodes = 2;
if SAVE_FILE
    file_data = num_nodes;
    dlmwrite(output_filename_nodes, file_data', 'delimiter',',','-append');
end

id_node = 1;
pose_x_node = 0.0; % index 1
pose_y_node = 0.0;
num_nodes_neighbours = 1;
id_nodes_neighbours = 2;
if SAVE_FILE
    file_data = [id_node pose_x_node pose_y_node num_nodes_neighbours id_nodes_neighbours];
    dlmwrite(output_filename_nodes, file_data', 'delimiter',',','-append');
end

id_node = 2;
pose_x_node = 100.9; % index 206
pose_y_node = 3.89;
num_nodes_neighbours = 1;
id_nodes_neighbours = 1;
if SAVE_FILE
    file_data = [id_node pose_x_node pose_y_node num_nodes_neighbours id_nodes_neighbours];
    dlmwrite(output_filename_nodes, file_data', 'delimiter',',','-append');
end
%--------------------------------------------------------------------------------------------------------------------
%--------------------------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------------------------
% define the links of grah-trajectory here
%--------------------------------------------------------------------------------------------------------------------
jump_in_meters = 2; %this is the parameter for subsampling

num_inks = 5;
if SAVE_FILE
    file_data = num_inks;
    dlmwrite(output_filename_links, file_data', 'delimiter',',','-append');
end

id_link = [1 2];
index_init = 1;
index_end = 206;
[pointsx_in_link, pointsy_in_link, n_points] = subsampling_traject(trajectory_x(index_init:index_end), trajectory_y(index_init:index_end), jump_in_meters);
if SAVE_FILE
    file_data = [id_link n_points pointsx_in_link pointsy_in_link];
    dlmwrite(output_filename_links, file_data', 'delimiter',',','-append');
end

id_link = [2 1];
index_init = 206;
index_end = 432;
[pointsx_in_link, pointsy_in_link, n_points] = subsampling_traject(trajectory_x(index_init:index_end), trajectory_y(index_init:index_end), jump_in_meters);
if SAVE_FILE
    file_data = [id_link n_points pointsx_in_link pointsy_in_link];
    dlmwrite(output_filename_links, file_data', 'delimiter',',','-append');
end
%--------------------------------------------------------------------------------------------------------------------
%--------------------------------------------------------------------------------------------------------------------

%--------------------------------------------------------------------------------------------------------------------
% define goals in grah-trajectory here
%--------------------------------------------------------------------------------------------------------------------
num_goals = 1;
if SAVE_FILE
    file_data = num_goals;
    dlmwrite(output_filename_goals, file_data', 'delimiter',',','-append');
end

id_goal = 1;
pose_x_goal = 0.0;
pose_y_goal = 0.0;
link_goal = [1 2];
if SAVE_FILE
    file_data = [id_goal pose_x_goal pose_y_goal link_goal];
    dlmwrite(output_filename_goals, file_data', 'delimiter',',','-append');
end
%--------------------------------------------------------------------------------------------------------------------
%--------------------------------------------------------------------------------------------------------------------


figure
hold on
grid
plot(trajectory_x, trajectory_y, '. b', 'linewidth', 2);
plot(pointsx_in_link, pointsy_in_link, '. r', 'linewidth', 5);
ylabel('y (m)');
xlabel('x (m)');
axis([min(trajectory_x)-10 max(trajectory_x)+10 min(trajectory_y)-10 max(trajectory_y)+10]);