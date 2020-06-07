function [junction_points, updated_map_robot] = CVD_rough_map(GVD, updated_map_robot)

% For testing purposes
%find junction points in image
point_intersection_map = bwmorph(GVD, 'branchpoints');
updated_map_robot(point_intersection_map == 1)=0.3;
% figure;
% imshow(updated_map_robot);
% hold on

[row, col] = find(updated_map_robot == 0.3);
junction_points = [col, row];

end

