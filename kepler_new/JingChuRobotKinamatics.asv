
%% bachmark
robot = importrobot('JingChuJointR.urdf');
%%
% robot.show();
% robot.showdetails();

% % tf = getTransform(robot, homeConfig, 'left_hip_roll')

%%
addpath('./modernRobot/mr')
addpath('./kepler_new')

%%
so3mat = VecToso3([0 0 1]) %[output:21e89597]
[omghat, theta] = AxisAng3([0 0 1]*3)

deg = 30;
Tsb = [cosd(deg) -sind(deg) 0 1;
       sind(deg) cosd(deg) 0 2;
       0 0 1 0;
       0 0 0 1];
se3mat = MatrixLog6(Tsb)
[S, theta] = AxisAng6(se3mat)


% OMGsc = 


%%


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":48.6}
%---
%[output:21e89597]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"函数或变量 'VecToso3' 无法识别。"}}
%---
