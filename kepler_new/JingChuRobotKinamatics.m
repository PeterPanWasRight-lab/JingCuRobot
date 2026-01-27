
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
syms L_1 L_2 L_3 theta_3 real positive
M = [eye(3) [L_1+L_2+L_3 0 0 ]';
     0 0 0 1];

S3 = [0 0 1 0 -(L1+L2) 0 ]';
S3_se3 = VecTose3(S3) %[output:7fa970f3]
T = MatrixExp6(S3_se3*theta_3) %[output:57e1a816]

T*M %[output:18196079]



%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":48.6}
%---
%[output:7fa970f3]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"S3_se3","rows":4,"type":"double","value":[["0","-1","0","0"],["1","0","0","-7"],["0","0","0","0"],["0","0","0","0"]]}}
%---
%[output:57e1a816]
%   data: {"dataType":"symbolic","outputData":{"name":"T","value":"\\left(\\begin{array}{cccc}\n\\cos \\left(\\theta_3 \\right) & -\\sin \\left(\\theta_3 \\right) & 0 & 7-7\\,\\cos \\left(\\theta_3 \\right)\\\\\n\\sin \\left(\\theta_3 \\right) & \\cos \\left(\\theta_3 \\right) & 0 & -7\\,\\sin \\left(\\theta_3 \\right)\\\\\n0 & 0 & 1 & 0\\\\\n0 & 0 & 0 & 1\n\\end{array}\\right)"}}
%---
%[output:18196079]
%   data: {"dataType":"symbolic","outputData":{"name":"ans","value":"\\left(\\begin{array}{cccc}\n\\cos \\left(\\theta_3 \\right) & -\\sin \\left(\\theta_3 \\right) & 0 & \\cos \\left(\\theta_3 \\right)\\,{\\left(L_1 +L_2 +L_3 \\right)}-7\\,\\cos \\left(\\theta_3 \\right)+7\\\\\n\\sin \\left(\\theta_3 \\right) & \\cos \\left(\\theta_3 \\right) & 0 & \\sin \\left(\\theta_3 \\right)\\,{\\left(L_1 +L_2 +L_3 \\right)}-7\\,\\sin \\left(\\theta_3 \\right)\\\\\n0 & 0 & 1 & 0\\\\\n0 & 0 & 0 & 1\n\\end{array}\\right)"}}
%---
