mex planner.cpp

startQ = [pi/2 pi/2 pi/2 pi/4 pi/2];
goalQ = [pi/8 3*pi/4 pi 0.9*pi 1.5*pi];
%1  BR->TL
startQ =[startQ;[pi/8 3*pi/4 pi 0.9*pi 1.5*pi]];
goalQ = [goalQ; [pi/2 pi/2 pi/2 pi/4 pi/2]];
%2
startQ =[startQ;[1.5708 1.5000 2.2000 2.8274 1.5000]];
goalQ = [goalQ; [1.5708 1.5708 1.5708 0.7854 0.4000]];
%3  BL->BR
startQ = [startQ; [1.5708 1.5708 1.5708 0.7854 0.4000]];
goalQ =[goalQ;[1.5708 1.5000 2.2000 2.8274 1.5000]];
%4
startQ = [startQ; [1.5708    1.0000    0.2000    5.5000    4.5000]];
goalQ =[goalQ;[1.5708    1.5708    1.5708    0.7854    0.4000]];
%5   TR->BR
startQ = [startQ; [1.5708    1.5708    1.5708    0.7854    0.4000]];
goalQ =[goalQ;[1.5708    1.0000    0.2000    5.5000    4.5000]];
%6
startQ = [startQ; [1.5708    1.0000    0.2000    5.5000    4.5000]];
goalQ =[goalQ;[pi/8 3*pi/4 pi 0.9*pi 1.5*pi]];
%7  TR -> TL
startQ = [startQ; [pi/8 3*pi/4 pi 0.9*pi 1.5*pi]];
goalQ =[goalQ;[1.5708    1.0000    0.2000    5.5000    4.5000]];
%8
startQ = [startQ; [1.5708    1.0000    0.2000    5.5000    4.5000]];
goalQ =[goalQ;[1.5708 1.5000 2.2000 2.8274 1.5000]];
%9  TR->BL
startQ = [startQ; [1.5708 1.5000 2.2000 2.8274 1.5000]];
goalQ =[goalQ;[1.5708    1.0000    0.2000    5.5000    4.5000]];
%10
startQ = [startQ; [pi/8 3*pi/4 pi 0.9*pi 1.5*pi]];
goalQ =[goalQ;[1.5708 1.5000 2.2000 2.8274 1.5000]];
%11  TL-> BL
startQ = [startQ; [1.5708 1.5000 2.2000 2.8274 1.5000]];
goalQ =[goalQ;[pi/8 3*pi/4 pi 0.9*pi 1.5*pi]];
%12
startQ = [startQ; [pi/8 3*pi/4 pi 0.9*pi 1.6*pi]];
goalQ =[goalQ;[1.5708 1.5708 1.5708 0.7854 0.4000]];
%13 TL->BR
startQ = [startQ; [1.5708 1.5708 1.5708 0.7854 0.4000]];
goalQ =[goalQ;[pi/8 3*pi/4 pi 0.9*pi 1.6*pi]];
%14
startQ = [startQ; [1.5708 2.000 0.7000 0.2 0]];
goalQ =[goalQ;[1.5708 1.5708 1.5708 0.7854 0.4000]];
%15 MR->BR
startQ = [startQ; [1.5708 1.5708 1.5708 0.7854 0.4000]];
goalQ =[goalQ;[1.5708 2.000 0.7000 0.2 0]];
%16
startQ = [startQ; [pi/8 3*pi/4 pi 0.9*pi 1.6*pi]];
goalQ =[goalQ;[1.5708 2.000 0.7000 0.2 0]];
%17 MR->TL
startQ = [startQ; [1.5708 2.000 0.7000 0.2 0]];
goalQ =[goalQ;[pi/8 3*pi/4 pi 0.9*pi 1.6*pi]];
%18
startQ = [startQ; [1.5708 1.5000 2.2000 2.8274 1.5000]];
goalQ =[goalQ;[1.5708 2.000 0.7000 0.2 0]];
%19 MR->BL
startQ = [startQ; [1.5708 2.000 0.7000 0.2 0]];
goalQ =[goalQ;[1.5708 1.5000 2.2000 2.8274 1.5000]];
%20
for i = 0:3
    fprintf('planner_id: %i\n', i)
    for j = 1:20
        fprintf('runtest: %i\n', j)
        runtest('map2.txt',startQ(j,1:end), goalQ(j,1:end), i);
    end
end