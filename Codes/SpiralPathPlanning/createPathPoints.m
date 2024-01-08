function points = createPathPoints(numberOfSides)
    points = [];
    center = [0 0];
    interconnect = 1;
    for i=3:-0.5:1
        polygon = nsidedpoly(numberOfSides,'Center',center,'SideLength',i);
        points = [points;polygon.Vertices];
        points = [points;[points(interconnect,1) points(interconnect,2)]];
        interconnect = interconnect + numberOfSides + 1;
    end
end
