load exampleMaps
map = binaryOccupancyMap(complexMap);
figure(1)
hold on
show(map)
mapInflated = copy(map);
RobotWidth = 1;
inflate(mapInflated, RobotWidth);
prm = robotics.PRM(mapInflated);
prm.NumNodes = 500;
prm.ConnectionDistance = 10;
startLocation = [4.0 2.0];
endLocation = [48.0 35.0];
path = findpath(prm, startLocation, endLocation);
show(prm);