function [roadmap] = create_roadmap(GVD, dist, robot_map, delaunay_triangulation, junction_points)

[GVD_y,GVD_x] = find(GVD==1);
GVD_points= [GVD_x,GVD_y];
%find corridors
corridors = (delaunay_triangulation(:,:,1)==1);
%place bounding boxes on corridor points
boxes = regionprops(corridors,'BoundingBox','Centroid');
rescale_dist = rescale(dist);% rescale to 0 or 1.
boundingboxes = cat(1, boxes.BoundingBox);
%centroids = cat(1,boxes.Centroid);
centroids =[];
closest_feature_point =[];
%for all bounding boxes find the GVD points in between. From those points
%find which have the lowest value in the distance transform of image. Those
%points are the closest features between obstacles. Accumalate the points
%and join them with surrounding junction vertices by checking which
%junction vertices are closer to the center of the bounding box.
for i=1:length(boxes)
    box= boxes(i).BoundingBox;
    x=box(1,1);
    y=box(1,2);
    width=box(1,3);
    height=box(1,4);
    if width<250
        filtered_points = GVD_points(GVD_points(:,1)> x & GVD_points(:,2)>y & GVD_points(:,1)<x+width & GVD_points(:,2)<y+height,:);
        size(filtered_points)
        filtered_dist=[];
        dim = size(filtered_points);
    for l=1:dim(1)
        filtered_dist= [filtered_dist; rescale_dist(filtered_points(l,2),filtered_points(l,1))];
    end
    size(filtered_dist);
    [min_dist, ind] = min(filtered_dist);
    ind = find(filtered_dist==min_dist);
    centroid = [ x+(width/2) y+(height/2)];
    %closest junction points
    ind1 = knnsearch(junction_points,centroid,'K',2,'Distance','euclidean');
    point1 = junction_points(ind1(1,1),:);
    point2 = junction_points(ind1(1,2),:);
    %check if the closest pairs of points are many. THis means its a line
    %(bisector)
    if length(ind)>1 & ~(isempty(filtered_points))
        ind = [ind(1,1); ind(length(ind),1)];
        robot_map = insertShape(robot_map,'Line',[filtered_points(ind(1),:) filtered_points(ind(2),:) ],'Color',[0.7 0.7 0.7]);
        if pdist([point1;filtered_points(ind(1),:)],'euclidean') < pdist([point1;filtered_points(ind(2),:)],'euclidean')
            robot_map = insertShape(robot_map,'Line',[point1 filtered_points(ind(1),:)],'Color',[0.7 0.7 0.7]);
            robot_map = insertShape(robot_map,'Line',[filtered_points(ind(2),:) point2],'Color',[0.7 0.7 0.7]);
        else
            robot_map = insertShape(robot_map,'Line',[point2 filtered_points(ind(1),:)],'Color',[0.7 0.7 0.7]);
            robot_map = insertShape(robot_map,'Line',[filtered_points(ind(2),:) point1],'Color',[0.7 0.7 0.7]);
        end
            
    elseif  ~(isempty(filtered_points))
        robot_map = insertShape(robot_map,'Line',[point1 filtered_points(ind,:)],'Color',[0.7 0.7 0.7]);
        robot_map = insertShape(robot_map,'Line',[filtered_points(ind,:) point2],'Color',[0.7 0.7 0.7]);
        
    end
    
    % What are these closest_feature_point and filtered_point?
    filtered_point=filtered_points(ind,:);    %filtered_points
    if ~(isempty(filtered_point))
        closest_feature_point = [closest_feature_point; filtered_point];
        centroids = [centroids; x+(width/2) y+(height/2)];
    end
    end
end

 
%connect closest junction points
[ind,min_dists] = knnsearch(junction_points,junction_points,'K',2,'Distance','euclidean');
for i=1:length(ind)
    if min_dists(i,2)<30
        point = junction_points(i,:);
        point1 = junction_points(ind(i,1),:);
        point2 = junction_points(ind(i,2),:);
        %point3 = closest_feature_point(ind(i,3),:);
        %robot_map = insertShape(robot_map,'Line',[point point1],'Color','blue');
        robot_map = insertShape(robot_map,'Line',[point point2],'Color',[0.7 0.7 0.7]);
        %robot_map = insertShape(robot_map,'Line',[point point3],'Color','blue');
    end
end
% % plot(centroids(:,1),centroids(:,2),'b*')
% imshow(robot_map);
% hold on;
% plot(junction_points(:,1),junction_points(:,2),'g*');
%  plot(closest_feature_point(:,1),closest_feature_point(:,2),'r*');
%   plot(centroids(:,1),centroids(:,2),'b*')

roadmap = robot_map(:,:,1); 



hold off



