function [mink_map] = minkowski(side_length, clearance, map)

[m,n] = size(map);

robotRad = side_length/sqrt(2) + clearance;

% Enlarging the boundaries of the map.
map(2+ round(side_length/2),round(side_length/2)+10:n-round(side_length/2)-10)=0;
map(m - round(side_length/2)-1,round(side_length/2)+10:n-round(side_length/2)-10)=0;
map(round(side_length/2)+10:m-round(side_length/2)-10,2+round(side_length/2))=0;
map(round(side_length/2)+10:m-round(side_length/2)-10,n-round(side_length/2)-1)=0;
%map(:,n-round(side_length/2))=0;

mink_map = map;


for i=1+round(side_length/2):m - round(side_length/2)
    for j=1+round(side_length/2):n - round(side_length/2)
        if map(i,j) == 0
            %mink_map = insertShape(mink_map,'FilledCircle',[j, i, robotRad], 'Color', 'black');
            %rectangle('Position',[j-(side_length/2),i-(side_length/2),side_length, side_length],'Edgecolor', [0,0,0]);
            mink_map(i-round(side_length/2):i+ round(side_length/2),j-round(side_length/2):j+round(side_length/2))=0;
        end
    end
end


end