function visualize_inertia_ellipsoid(mass, com, I_global, varargin)
    % VISUALIZE_INERTIA_ELLIPSOID 可视化惯性椭球
    %
    % 输入参数:
    %   mass     : 标量, 质量 (kg)
    %   com      : 1x3 或 3x1 向量, 质心在世界坐标系下的位置 [x, y, z]
    %   I_global : 3x3 矩阵, 世界坐标系下的惯性张量 (I_global = R * I_local * R')
    %   varargin : 可选参数，用于设置颜色等
    
    % 确保 com 是行向量
    com = reshape(com, 1, 3);
    
    %% 1. 计算椭球几何参数
    % 对全局惯性张量进行特征分解
    % V: 特征向量矩阵 (代表椭球的主轴方向，即旋转矩阵)
    % D: 特征值对角阵 (代表主惯性矩 I1, I2, I3)
    [V, D] = eig(I_global);
    I_princ = diag(D); % 提取主惯性矩
    
    % 防止数值误差导致的极小负数
    I_princ = abs(I_princ);
    
    % 物理公式反解半轴长度 (a, b, c)
    % 对于均匀椭球: I1 = m/5 * (b^2 + c^2) ...
    % 反解: a^2 = 2.5/m * (I2 + I3 - I1)
    % 注意: 特征值顺序可能与 x,y,z 不对应，但与特征向量 V 的列一一对应
    
    factor = 2.5 / mass;
    
    % 半轴长平方 (对应 V 的第1, 2, 3列方向)
    sq_radii = [
        factor * (I_princ(2) + I_princ(3) - I_princ(1));
        factor * (I_princ(1) + I_princ(3) - I_princ(2));
        factor * (I_princ(1) + I_princ(2) - I_princ(3))
    ];

    % 开根号得到半轴长，加 max 防止非物理数据导致复数
    radii = sqrt(max(sq_radii, 1e-9)); 
    
    %% 2. 生成几何网格
    [X, Y, Z] = sphere(30); % 生成单位球
    
    % 缩放 (Scale)
    X = X * radii(1);
    Y = Y * radii(2);
    Z = Z * radii(3);
    
    % 旋转 (Rotate)
    % 将网格点转换为 3xN 矩阵进行旋转
    pts = [X(:)'; Y(:)'; Z(:)'];
    pts_rot = V * pts; % 应用特征向量矩阵作为旋转
    
    % 平移 (Translate) 并重构回网格形状
    X_final = reshape(pts_rot(1,:), size(X)) + com(1);
    Y_final = reshape(pts_rot(2,:), size(Y)) + com(2);
    Z_final = reshape(pts_rot(3,:), size(Z)) + com(3);
    
    %% 3. 绘制惯性椭球
    % 保持您原本的风格：线框模式，橙色
    h_ellip = mesh(X_final, Y_final, Z_final, ...
         'EdgeColor', [0.8500 0.3250 0.0980], ... % 橙色
         'FaceColor', 'none', ...
         'FaceAlpha', 0);
     
    %% 4. (可选) 绘制质量球
    % 为了保持直观，保留原来的质量大小可视化
    radius_mass = 0.04 * (mass^(1/3)); 
    [Xs, Ys, Zs] = sphere(15);
    Xs = Xs * radius_mass + com(1);
    Ys = Ys * radius_mass + com(2);
    Zs = Zs * radius_mass + com(3);
    
    h_mass = surf(Xs, Ys, Zs, ...
        'FaceColor', 'c', ... % 青色
        'EdgeColor', 'none', ...
        'FaceAlpha', 0.3);
    
    %% 5. 标注文本
    % 稍微偏移一点 z 轴显示文字
    text(com(1), com(2), com(3) + max(radii)*1.1, ...
        sprintf('m=%.1f', mass), 'FontSize', 8, 'Color', 'k', 'HorizontalAlignment', 'center');
    
    % 设置 Tag 方便后续查找
    set(h_ellip, 'Tag', 'InertiaEllipsoid');
    set(h_mass, 'Tag', 'MassSphere');
end