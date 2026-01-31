clc; clear; close all;

% 1. 设置画图画布
figure('Color', 'w');
hold on; axis equal; grid on;
axis([-0.5 1.5 -1 1]);
title('李群 (圆) vs 李代数 (切线)');
xlabel('x'); ylabel('y');

% 2. 绘制李群 (SO(2) 在几何上就是一个单位圆)
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'b-', 'LineWidth', 2); 
text(0, 1.1, '李群 (流形)', 'Color', 'b');

% 3. 绘制单位元 (Identity)
plot(1, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
text(1.05, 0.05, '单位元 I', 'Color', 'r');

% 4. 绘制李代数 (切空间)
% 在 SO(2) 中，单位元 (1,0) 处的切线是垂直线 x=1
y_tan = linspace(-1, 1, 100);
x_tan = ones(size(y_tan));
plot(x_tan, y_tan, 'k--', 'LineWidth', 1.5);
text(1.05, 0.8, '李代数 (切空间)', 'Color', 'k');

% 5. 演示：代数上的向量 vs 群上的运动
% 定义一个李代数元素 (反对称矩阵，代表角速度)
omega = 1; % 标量值
X = [0 -omega; omega 0]; % 生成元 (Generator)

% 模拟时间 t 的变化
dt_steps = linspace(0, 0.8, 20); 

for t = dt_steps
    % --- A. 在李代数上 (直线走) ---
    % 这里的 v 就是切向量 v = X * t 的某种表示
    % 在几何图中，它就是切线上的高度
    linear_pos = [1; omega * t]; 
    h1 = plot(linear_pos(1), linear_pos(2), 'm.', 'MarkerSize', 15);
    
    % --- B. 在李群上 (通过指数映射弯曲回去) ---
    % R = expm(X * t)
    R = expm(X * t);
    % 作用在基向量 [1;0] 上看位置
    group_pos = R * [1; 0];
    h2 = plot(group_pos(1), group_pos(2), 'g.', 'MarkerSize', 15);
    
    % 画出对应关系
    h3 = plot([linear_pos(1) group_pos(1)], [linear_pos(2) group_pos(2)], 'k:');
    
    pause(0.1);
    if t ~= dt_steps(end)
        delete(h1); delete(h2); delete(h3);
    end
end

legend('群流形', '单位元', '李代数切线', '代数上的点', '映射回群的点');