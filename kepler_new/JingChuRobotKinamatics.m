
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
syms L_1 L_2 L_3 theta_3 real positive %[output:7fa970f3] %[output:57e1a816] %[output:18196079]
M = [eye(3) [L_1+L_2+L_3 0 0 ]';
     0 0 0 1];

S3 = [0 0 1 0 -(L1+L2) 0 ]';
S3_se3 = VecTose3(S3)
T = MatrixExp6(S3_se3*theta_3)

T*M
%%
% UR5
% W = [0.109 0.082];
% L = [0.425 0.392]; 
% H = [0.089 0.095];

W = sym("W",[1,2],"real");
L = sym("L",[1,2],"real");
H = sym("H",[1,2],"real");

M = [-1 0 0 0; %[output:group:566f76b7] %[output:54c15239]
     0 0 1 0; %[output:54c15239]
     0 1 0 0; %[output:54c15239]
     L(1)+L(2) W(1)+W(2) H(1)-H(2) 1 %[output:54c15239]
     ]' %[output:group:566f76b7] %[output:54c15239]
%% vs6 有两种求解方式：
% 一种直接用定义求(-omg x p06) ，一种用虚拟刚体在原点的速度求。
% 第一种更容易程式化, 而且 坐标系的原点位置可以在整个转轴上随便选择 为了后面的动力学，可以尽量选择和URDF匹配的坐标系
% vs6 = cross([0 1 0]',[-(L(1)+L(2)), 0 ,H(2)-H(1)]')
% 为什么可以在轴线上任选坐标系原点位置？数学上很好看出来，叉乘消去一个维度。物理意义在于，一旦确定好M矩阵，那么末端坐标系都确定了，所有的旋转都是按照轴线去改变M矩阵代表的坐标系的位置，和前序的坐标系位置在轴线的哪里无关
% 无论旋量轴上的坐标系选择在什么地方，最终都可以通过一次旋转，将末端坐标系（M）转移到想要的位置（这就是PoE建模的坐标系无关特性）
% 注意注意  这种无关性只适用于旋转轴关节机械臂？no 移动关节也适用
%% 这种无关性不是"bug"，而是螺旋理论的一个"feature"——它反映了刚体运动的本质特性，即刚体运动由旋转轴和螺距（纯转动关节h=0）完全确定，与具体的参考点选择无关。

%% 搞清楚螺旋轴的正方向（和实际电机转动方向一致）
% 对于旋转关节机械臂上的旋量而言，节距h=0。如果存在节距，则不符合实际物理意义(如果h不等于0，则会发生坐标系的螺旋进动，实际中不能有）

%% 反向倒推法
omgs6 = [0 1 0]';
p06 = [(L(1)+L(2)), W(1)+W(2) ,H(1)-H(2)]' %[output:86b824cc]
vs6 = cross(omgs6, -p06); % 可以发现W维度的信息因为叉乘而被消去了
S6 = [omgs6; vs6]; %[output:14fbd594]

omgs5 = []




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
%[output:54c15239]
%   data: {"dataType":"symbolic","outputData":{"name":"M","value":"\\left(\\begin{array}{cccc}\n-1 & 0 & 0 & L_1 +L_2 \\\\\n0 & 0 & 1 & W_1 +W_2 \\\\\n0 & 1 & 0 & H_1 -H_2 \\\\\n0 & 0 & 0 & 1\n\\end{array}\\right)"}}
%---
%[output:86b824cc]
%   data: {"dataType":"symbolic","outputData":{"name":"p06","value":"\\left(\\begin{array}{c}\nL_1 +L_2 \\\\\nW_1 +W_2 \\\\\nH_1 -H_2 \n\\end{array}\\right)"}}
%---
%[output:14fbd594]
%   data: {"dataType":"symbolic","outputData":{"name":"S6","value":"\\left(\\begin{array}{c}\n0\\\\\n1\\\\\n0\\\\\nH_2 -H_1 \\\\\n0\\\\\nL_1 +L_2 \n\\end{array}\\right)"}}
%---
