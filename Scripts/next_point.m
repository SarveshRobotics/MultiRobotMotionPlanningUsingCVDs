function [point, x, y] = next_point(start_point, x, y, end_point)
distance= sqrt((x-start_point(:,1)).^2 + (y-start_point(:,2)).^2);

% distance1 = distance(distance>0);% Distance will always be positive!
index = find(distance <= 5);

% Possible candidates coordinates
candidates=[x(index,1) y(index,1)];

% Assigning value of 0 to (x,y) where it corresponding coordinates had distance less than 5
x(index,1)=0;% so that we don't check them again!
y(index,1)=0;

% How many candidates did we find?
s=size(candidates)

% If candidates are more than one?
if(s(:,1) > 1)
    % We will chose that coordinate among the candidates that has minimum
    % distance from the goal point.
    end_dist = sqrt((candidates(:,1)-end_point(:,1)).^2 + (candidates(:,2)-end_point(:,2)).^2);
    
    [minimum_distance_found, ind] = min(end_dist);
    
    % Retreiving the next point we should move to.
    point=[candidates(ind,1) candidates(ind,2)];
else
%     % Otherwise just select the only candidate.
%     try
        point=[candidates(1,1) candidates(1,2)];
%     catch
%         disp('Error in Candidates');
%         disp(candidates);
%         disp('Error in Candidates');
%         point = [candidates(1,1) candidates(1,2)];
%     end
end
end 


