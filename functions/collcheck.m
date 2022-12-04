function coll = collcheck(map, x, y, d)
    
    [size_x, size_y] = size(map);
    %x-d
    %x+d
    %y-d
    %y+d
    if x-d < 1 || x+d > size_x || y-d < 1 || y+d > size_y
        coll = -100;
    else
        coll = sum(map(x-d:x+d, y-d:y+d),'all');
    end
end