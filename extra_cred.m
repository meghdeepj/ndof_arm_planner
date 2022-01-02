mex -g planner.cpp

startQ = [pi/8 3*pi/4 pi 0.9*pi 1.5*pi];
goalQ =[1.5708 1.5000 2.2000 2.8274 1.5000];
planner_id = 2;
runtest('map2.txt',startQ, goalQ, planner_id);