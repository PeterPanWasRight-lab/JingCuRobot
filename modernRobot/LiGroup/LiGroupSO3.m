clc; clear; close all;

% 1. 设置画布
figure('Color', 'w', 'Position', [100, 100, 800, 600]);
axis equal; grid on; hold on;
view(135, 30); % 调整视角
xlabel('X'); ylabel('Y'); zlabel('Z');
title('SO(3): 李代数(红轴) 驱动 李群(蓝点轨迹)');

% 画一个半透明的单位球作为参考背景
[sx, sy, sz] = sphere(30);
surf(sx, sy, sz, 'FaceColor', 'white', 'EdgeAlpha', 0.1, 'FaceAlpha', 0.1);

% ---------------------------------------------------------
% 2. 定义李代数元素 (Lie Algebra)
% ---------------------------------------------------------
% 在 SO(3) 中，切向量就是角速度向量 omega
% 我们随便选一个方向：同时绕 X轴 和 Z轴 旋转
omega_vec = [1; 0; 2]; 
omega_vec = omega_vec / norm(omega_vec); % 归一化，作为旋转轴方向

% 将向量转化为反对称矩阵 (Hat Map)
% 这才是李代数 mathematically 的真身：Skew-symmetric matrix
omega_hat = [0, -omega_vec(3), omega_vec(2);
             omega_vec(3), 0, -omega_vec(1);
             -omega_vec(2), omega_vec(1), 0];

% 可视化李代数：画出这个旋转轴
quiver3(0, 0, 0, omega_vec(1)*1.5, omega_vec(2)*1.5, omega_vec(3)*1.5, ...
    'r', 'LineWidth', 3, 'MaxHeadSize', 0.5);
text(omega_vec(1)*1.6, omega_vec(2)*1.6, omega_vec(3)*1.6, ...
    '\leftarrow 李代数 (旋转轴)', 'Color', 'r', 'FontSize', 12);

% ---------------------------------------------------------
% 3. 定义李群上的初始点 (Identity 附近的参考点)
% ---------------------------------------------------------
% 假设球面上有一个点 P0 (比如在赤道上)
P0 = [1; 0; 0];
h_point = plot3(P0(1), P0(2), P0(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
h_trace = animatedline('Color', 'b', 'LineWidth', 2);

% ---------------------------------------------------------
% 4. 动画演示：指数映射 (Exponential Map)
% ---------------------------------------------------------
% 这里的 t 就是步长，你可以理解为旋转的角度
total_angle = 2 * pi; 
steps = linspace(0, total_angle, 100);

for t = steps
    % A. 核心公式：指数映射 R = exp(omega_hat * t)
    % 只要知道了代数(omega_hat)和参数(t)，就能算出群元素(R)
    R = expm(omega_hat * t); 
    
    % B. 应用群元素：把点 P0 旋转到新位置
    P_new = R * P0;
    
    % C. 更新绘图
    set(h_point, 'XData', P_new(1), 'YData', P_new(2), 'ZData', P_new(3));
    addpoints(h_trace, P_new(1), P_new(2), P_new(3));
    
    % 动态显示切速度向量 (Velocity Vector)
    % 速度 v = omega x r
    % 这展示了李代数如何在每一点产生"切向速度"
    v = cross(omega_vec, P_new); 
    % 画出瞬时速度箭头 (绿色)
    % 注意：这个绿色箭头始终与蓝色轨迹相切！
    if exist('h_vel', 'var'), delete(h_vel); end
    h_vel = quiver3(P_new(1), P_new(2), P_new(3), v(1), v(2), v(3), ...
        0.5, 'g', 'LineWidth', 2);
    
    drawnow;
    pause(0.02);
end

legend('单位球', '李代数 (轴)', '当前位置', '群轨迹 (圆弧)');