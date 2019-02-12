function  [subsampled_traject_x, subsampled_traject_y, n_sub] = subsampling_traject(original_traject_x, original_traject_y, jump_in_meters)
% In this function we get a entire trayectory and subsample this

n        = length(original_traject_x);
n_sub = 0;

subsampled_traject_x = [];
subsampled_traject_y = [];

n_sub = 1;
subsampled_traject_x = [subsampled_traject_x, original_traject_x(1)];
subsampled_traject_y = [subsampled_traject_y, original_traject_y(1)];

for i = 2:n
    pose_x = original_traject_x(i);
    pose_y = original_traject_y(i);
    d_eucidean = sqrt((pose_x-subsampled_traject_x(end))^2 + (pose_y-subsampled_traject_y(end))^2);
    if d_eucidean >= jump_in_meters
        n_sub = n_sub +1;
        subsampled_traject_x = [subsampled_traject_x, pose_x];
        subsampled_traject_y = [subsampled_traject_y, pose_y];
    end
end

end

