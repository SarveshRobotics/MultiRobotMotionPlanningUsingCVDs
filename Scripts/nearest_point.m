function [closest_point] = nearest_point( point,x,y )
% Returns the closest point in the moveable workspace to a given point.

% Calculating the distance of every point in movable workspace to the given point
distance= sqrt((x-point(:,1)).^2 + (y-point(:,2)).^2);

% Returning the index of the closest index
[minimum_distance_found,index] = min(distance);

% disp('I am the closest point: ');
% disp(index);
closest_point=[x(index,1) y(index,1)];
end

