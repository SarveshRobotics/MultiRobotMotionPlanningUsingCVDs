close all
clear all
clc
%% Definitions
size_of_robot = 5;
clearance = 2;
CVD_robots = [];
%no_of_obstacles=input('Enter no. of obstacles: ');
no_of_robots=input('Enter no. of robots: ');
space=ones(500,500); % Create our Workspace
hfig=figure(1); % Plot the Workspace
imshow(space,[]); % Show the Workspace

%disp('Draw polygons on figure. Double click on any vertex of polygon to fill and complete it: ');


%% Creating Workspace Maps
% Drawing Obstacles
% for i=1:no_of_obstacles
%     obstacle=roipoly(space);
%     space=space-obstacle;
% end
%save('obstacle_space.mat','space');
space = load('obstacle_space.mat');
space = space.space;
imshow(space,[]); % Show updated Workspace

hold on;

% Robot Start and end Positions.
robots_start = [];
robots_end = [];

robot_size = 5;% Change Robot's size.
t=1;% Time Instance.

% Run for each Robot; Find what is each robot's start and end node.
for i = 1:no_of_robots 
    % What is the starting point for each robot?
    disp('Select start point: ');
    start_point = ginput(1);
    space_dim = size(space);
    
    % Did you input correct start point?
    while(~(start_point(1)+robot_size <= space_dim(1) && start_point(2)+robot_size <=space_dim(2) && start_point(1)-robot_size>=1 && start_point(2)-robot_size>=1))
        disp('Wrong input, Enter start point again')
        start_point= ginput(1);
    end
    
    % Keeping track of where each robot started!
    robots_start = [robots_start; start_point];% Appending each robot start's point to robot_start array.
    
    % Displaying each Robot with a red rectangle of size 5.
    rectangle('Position',[start_point(1)-robot_size, start_point(2)-robot_size, 2*robot_size, 2*robot_size], 'EdgeColor', 'r', 'FaceColor', 'r');
    
    % What is the ending point for this robot?
    disp('Select end point: ');
    end_point = ginput(1);
    
    % Did you input the correct end point?
    while(~(end_point(1)+robot_size <= space_dim(1) && end_point(2)+robot_size <=space_dim(2) && end_point(1)-robot_size>=1 && end_point(2)-robot_size>=1))
        disp('Wrong input, Enter end point again')
        end_point= ginput(1);
    end
    
    % Keeping track of where each robot has its goal!
    robots_end = [robots_end; end_point];% Appending end_point of each robot to the global array robots_end.
    
    % Display the start and end goal for this robot.
    plot(end_point(:,1),end_point(:,2),'g--o');
end

% Each robot now maintains copy of other robot's starting and goal points.
% for l=1:1
%hold off
%initialize current position to start position and flags
curr_pos=robots_start;
GVD_start_flags= zeros(no_of_robots,1);
GVD_end_flags=zeros(no_of_robots,1);
goal_complete_flags=zeros(no_of_robots,1);
%flag for simulation end
end_sim = 0;
%till simulation has not ended
while(end_sim==0)
    for i=1:no_of_robots
        %consider the positions of the other robot
        other_robots_pos = curr_pos;
        other_robots_pos(i,:)=[]; % Remove its starting point.
        robot_map = space;
        dim = size(other_robots_pos);% How many other robots there are?
    
    % Update current robot map!
        for j=1:dim(1)
        % Other robots are updated to be obstacles as well. Assigning
        % thier rectangular portion as zero.
            robot_map(other_robots_pos(j,2)-robot_size:other_robots_pos(j,2)+robot_size,other_robots_pos(j,1)-robot_size:other_robots_pos(j,1)+robot_size)=0;
        end
    
        % Update the Global Maps Array: robot_maps.
        robot_maps(:,:,t,i)=robot_map;
%% Creating Compact Voronoi Diagrams
%minkowski space for obstacle space
        updated_map(:,:,t,i) = minkowski(size_of_robot, clearance, robot_maps(:,:,t,i));
        updated_map_robot = updated_map(:,:,t,i);
%       figure;
%       hold on;
    %   GVD_temp = bwmorph(updated_map_robot,'remove');
        GVD_temp = updated_map_robot; 
        %for l=1:3
        %Use distance transform to compute GVD
        updated_map_robot_temp = bwdist(1-GVD_temp);
        %GVD_temp = imbinarize(rescale(updated_map_robot_temp),'adaptive');
        %end
        %consider the second derivative of the image of distance trnasform
        %to extract gvd
        [GVD]= rescale(del2(updated_map_robot_temp))<0.5;
        %Remove spurious pixels
        GVD = bwmorph(GVD,'spur');
        GVD = bwmorph(GVD,'thin');
        GVD = bwmorph(GVD,'clean');
        GVD=bwareaopen(GVD,20);
        %find the junction points
        [junction_points, updated_map_robot] = CVD_rough_map(GVD, updated_map_robot);
        %get delaunay triangulation, witness circles and remove spurious
        %centers
        [delaunay_triangulation,junction_points,witness_circle_radii,removed_centers] = create_witness_circles(junction_points,updated_map_robot);
        GVD(removed_centers) = 1;

%       figure;
%       imshow(delaunay_triangulation);
%       hold on;
        %create roadmap from the triangulation adn GVD
        roadmap = create_roadmap(GVD, updated_map_robot_temp, updated_map_robot, delaunay_triangulation, junction_points);
       %updated_map_robot(GVD==1)=0.7;
       %roadmap=updated_map_robot;
        %         figure;
%         imshow(roadmap);
%       hold on;
    
        roadmaps(:,:,t,i) = roadmap;
        
        %get pixels corresponding to the roadmap
        [y,x]=find(~(roadmap==0 | roadmap==1));% y - row; x - column.


%consider the end point of robot
        end_point = robots_end(i,:);
        %find closest point on roadmap to the goal
        GVD_end = nearest_point(end_point,x,y);
        %reach to closest point on road map from start position by
        %traversing on the line joining nearest point on roadmap to start
        %point
        if GVD_start_flags(i,1)==0
            [GVD_start] = nearest_point(curr_pos(i,:), x, y);
            if abs((GVD_start(1,1)-curr_pos(i,1)))>0.01
                slope=(GVD_start(1,2)-curr_pos(i,2))/(GVD_start(1,1)-curr_pos(i,1));
                c=curr_pos(i,2)- slope*curr_pos(i,1);
                if(curr_pos(i,1) < GVD_start(1,1))
                    k = 1;
                else
                    k = -1;
                end
                curr_pos(i,:) =  [curr_pos(i,1)+k, slope*(curr_pos(i,1)+k)+c];
            else
                 if(curr_pos(i,1) < GVD_start(1,1))
                    k = 1;
                else
                    k = -1;
                end
                curr_pos(i,:)=[curr_pos(i,1), curr_pos(i,2)+k];
            end
                if pdist([curr_pos(i,:);GVD_start],'euclidean')<0.1
                    GVD_start_flags(i,1)=1;
                end
                %traverse on the roadmap till the closes point on roadmap
                %to goal is reached
        elseif GVD_end_flags(i,1)==0
            [curr_pos(i,:), x, y] = next_point(curr_pos(i,:), x, y, GVD_end);
            if pdist([curr_pos(i,:);GVD_end],'euclidean')<0.1
                GVD_end_flags(i,1)=1;
            end
          %traverse line joining closest point on roadmap to goal, by traversing the line  
        elseif goal_complete_flags(i,1)==0
            if abs((curr_pos(i,1)-end_point(1,1)))>0.01
                slope=(curr_pos(i,2)-end_point(1,2))/(curr_pos(i,1)-end_point(1,1));
                c=curr_pos(i,2)- slope*curr_pos(i,1);
                if(end_point(1,1)<curr_pos(i,1))
                    k=-1;
                else
                    k=1;
                end
                curr_pos(i,:) = [curr_pos(i,1)+k, slope*(curr_pos(i,1)+k)+c];
            else
                if(end_point(1,1)<curr_pos(i,1))
                    k=-1;
                else
                    k=1;
                end
                curr_pos(i,:)=[curr_pos(i,1), curr_pos(i,2)+k];
            end
               %check if the goal has been reached
            if pdist([curr_pos(i,:);end_point],'euclidean')<1
                goal_complete_flags(i,1)=1;
            end

    
    

    
        end
        disp('Robot moving in figure!');
        figure(i+1)
        imshow(roadmap)
        hold on
        %plot robot positions on roadmap
        plot(curr_pos(i,1), curr_pos(i,2), 'b--O');
        pause(0.1);


    end
count = 0;
%     figure(2)
%     imshow(space)
%     hold on
%check if all robots have reached their respective goals
    for k = 1:no_of_robots
       % plot(curr_pos(k,1), curr_pos(k,2), 'b--O');
        if goal_complete_flags(k,1)==1
            count=count+1;
        end
    end
    if count== no_of_robots
        end_sim=1;
    end
end
% end