
function [delaunay_triangulation,junction_points,radii,removed_centers] = create_witness_circles(junction_points, updated_map_robot)

updated_map_robot_temp=updated_map_robot;
delaunay_triangulation = updated_map_robot;
B = bwboundaries(updated_map_robot);
%figure
%imshow(visboundaries(B));
% Converting cell array to matrix
B_all = cell2mat(B);

% Initializing the matrix which will store all the distances.
% Columns represent junction points, and rows represent the distance to
% each boundary point.
dist_matrix = zeros( length( B_all(:,1)), length(junction_points(:,1)) );

% Calculating the Euclidean distance
for i=1:length(junction_points(:,1))
    for j=1:length(B_all(:,1))
        dist = sqrt( (junction_points(i,1) - B_all(j,2))^2 + (junction_points(i,2) - B_all(j,1))^2 );
        dist_matrix(j, i) = dist;
    end
end

% Figuring out the minimum distance for each junction point
k = 3;
[dist_smallest,indices] = mink(dist_matrix, k, 1);


% % Plotting witness circles using those minimum distances
centers = [junction_points(:,1), junction_points(:,2)];
radii = (dist_smallest(1,:))';
removed_centers = centers(radii<=10,:);
centers = centers(radii>10,:);
radii=radii(radii>10,:);
junction_points=centers;




% Binary circle
% updated_map_robot_temp = insertShape(updated_map_robot_temp, 'Circle', [centers, radii], 'Color', 'black', 'Opacity', 1);
% imshow(updated_map_robot_temp);
% hold on
for i=1:length(junction_points)
    triangle_points=[];
    k=1;
    for j=1:length(B)
        object_boundary = B{j};
        object_boundary_temp = [object_boundary(:,2) object_boundary(:,1)];

        [ind,min_dist] = knnsearch(object_boundary_temp,junction_points(i,:));
        if abs(min_dist-radii(i)) <6
            triangle_points(k,:)=object_boundary_temp(ind,:);
            k=k+1;
        end
    end
[m,n]=size(triangle_points);
radii_temp = 5*ones(1,m);
triangle_temp = triangle_points';
triangle_temp = triangle_temp(:)';
size(triangle_temp)
if length(triangle_temp)==6
    delaunay_triangulation = insertShape(delaunay_triangulation, 'FilledPolygon', triangle_temp, 'Color', 'blue');
elseif length(triangle_temp)==4
    delaunay_triangulation = insertShape(delaunay_triangulation, 'Line', triangle_temp, 'Color', 'blue');
end
    
%viscircles(triangle_points, radii_temp, 'LineWidth', 1,'color','blue');
end


    
%figure;
%imshow(delaunay_triangulation);
% Geometric circle
radii = round(radii);
%viscircles(centers, radii, 'LineWidth', 1);
%viscircles(centers, radii-(radii-10), 'LineWidth', 1);



end







