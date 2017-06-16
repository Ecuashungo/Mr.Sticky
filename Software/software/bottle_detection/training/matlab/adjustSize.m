function [ newrect ] = adjustSize( rect,dim )

    newrect = round(rect);

    for j = 1:2
        if rect(j)<1
            newrect(j)=1;
            newrect(j+2) = rect(j+2)+rect(j)-1;
        elseif rect(j)+rect(j+2)>dim(j)
            newrect(j+2) = dim(j)-rect(j);
        end
    end 

end

