function [  ] = showRectangles( rect, currentBottle )
        
    hold on;

    for i = 1:size(rect,1)
        if i == currentBottle
            rectangle('Position',rect(i,:),'EdgeColor','r')
        else
            rectangle('Position',rect(i,:),'EdgeColor','g')
        end
    end
    
    hold off
    

end

