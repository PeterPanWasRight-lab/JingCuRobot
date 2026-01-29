
function visualizeInertia(mass, com, I_tensor, alpha, varargin)
% VISUALIZEINERTIA 可视化刚体的质心和惯量矩阵
%
%   输入参数:
%       mass     : 标量, 质量 (kg)
%       com      : 1x3 向量, 质心位置 [x, y, z]
%       I_tensor : 3x3 惯量矩阵 或 1x3 主惯量向量 [Ixx, Iyy, Izz]
%       alpha    : 盒子放缩系数（有时候盒子太小了看不到）
%       varargin : 可选参数, 'Name', Value 对
%                  'ShowAxes', true/false (默认 true) - 显示主轴
%                  'BoxAlpha', 0-1 (默认 0.35) - 惯量盒透明度
%                  'CoMRadius', scalar (默认 自动计算)
%
%   例子:
%       m = 5;
%       com = [1, 0, 0];
%       I = [0.1, 0, 0; 0, 0.2, 0; 0, 0, 0.2]; 
%       visualizeInertia(m, com, I);

    if length(varargin) < 4
        alpha = 1;
    end
    % --- 1. 参数解析与预处理 ---
    p = inputParser;
    addParameter(p, 'ShowAxes', true);
    addParameter(p, 'BoxAlpha', 0.3);
    addParameter(p, 'CoMRadius', []); % 默认根据盒子大小自动调整
    parse(p, varargin{:});
    opts = p.Results;

    % 确保 com 是列向量
    com = com(:);

    % 处理惯量张量
    if isvector(I_tensor) && length(I_tensor) == 3
        % 如果输入只是主惯量向量
        I_mat = diag(I_tensor);
    else
        I_mat = I_tensor;
    end

    % --- 2. 计算惯量盒尺寸与旋转 (对应 JS: MathUtils.computeInertiaBox) ---
    
    % 特征值分解：寻找主惯量轴和主惯量
    % V 是旋转矩阵 (特征向量), D 是对角矩阵 (特征值/主惯量)
    [R_rot, D_diag] = eig(I_mat);
    I_principal = diag(D_diag);
    
    % 确保旋转矩阵是右手系 (det = 1)。如果 det = -1，反转一列
    if det(R_rot) < 0
        R_rot(:,3) = -R_rot(:,3);
    end

    Ixx = I_principal(1);
    Iyy = I_principal(2);
    Izz = I_principal(3);

    % 计算等效长方体尺寸 (w, h, d)
    % 公式推导自: Ixx = m/12 * (h^2 + d^2), 等等。
    % 反解尺寸:
    % size_x (对应 Ixx 轴方向的长度) = sqrt(6 * (Iyy + Izz - Ixx) / m)
    % 注意: Ixx 是绕 X 轴旋转的惯量，取决于 Y(h) 和 Z(d) 的尺寸。
    % 所以 Box 的 X 尺寸由 Iyy 和 Izz 决定。
    
    size_x = sqrt(6 * (Iyy + Izz - Ixx) / mass);
    size_y = sqrt(6 * (Ixx + Izz - Iyy) / mass);
    size_z = sqrt(6 * (Ixx + Iyy - Izz) / mass);
    
    box_dims = real([size_x, size_y, size_z]); % 取实部防止数值误差导致的微小复数

    % 如果惯量数据不合法（违反三角不等式），尺寸可能为0或无效
    if any(box_dims <= 0)
        warning('惯量数据可能不物理 (违反三角不等式)，无法生成有效的几何盒子。');
    end

    % --- 3. 绘图准备 ---
    hold on;
    axis equal;
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    lighting gouraud;
    light('Position', [1 1 1], 'Style', 'infinite');
    view(3);

    % --- 4. 绘制惯量盒 (对应 JS: createInertiaEllipsoid) ---
    
    % 定义标准单位立方体的顶点 (8个)
    % 顺序与 patch 'Faces' 配合
    v = [ -1 -1 -1;  1 -1 -1;  1  1 -1; -1  1 -1; ...
          -1 -1  1;  1 -1  1;  1  1  1; -1  1  1 ] * alpha;
      
    % 缩放
    v = v .* box_dims; 
    
    % 旋转 (v * R_rot')
    v = (R_rot * v')';
    
    % 平移
    v = v + com';

    % 定义立方体的面 (6个面，每个面4个顶点)
    f = [ 1 2 3 4; 2 6 7 3; 6 5 8 7; 5 1 4 8; 4 3 7 8; 5 6 2 1 ];

    % 绘制 Patch
    % 颜色: 0x4a9eff (Light Blue) -> RGB [0.29, 0.62, 1.0]
    patch('Faces', f, 'Vertices', v, ...
          'FaceColor', [0.29, 0.62, 1.0], ...
          'FaceAlpha', opts.BoxAlpha, ...
          'EdgeColor', [0.1, 0.3, 0.6], ...
          'EdgeAlpha', 0.8, ...
          'LineWidth', 1.5);

    % --- 5. 绘制质心 CoM (对应 JS: createCOMGeometry) ---
    
    % 确定 CoM 半径
    if isempty(opts.CoMRadius)
        % 自动设定为盒子最大边长的 2%
        r = max(box_dims) * 0.02;
        if r == 0, r = 0.05; end
    else
        r = opts.CoMRadius;
    end

    % 生成球体数据
    [sx, sy, sz] = sphere(32); % 增加分段数以获得平滑效果
    
    % 生成黑白棋盘格纹理 (模拟 JS 中的 quarter sphere)
    % 逻辑：根据坐标的象限决定颜色
    colorData = zeros(size(sx));
    for i = 1:size(sx, 1)
        for j = 1:size(sx, 2)
            % 简单的象限逻辑来模拟黑白相间
            isWhite = xor(xor(sx(i,j)>0, sy(i,j)>0), sz(i,j)>0);
            if isWhite
                colorData(i,j) = 1; % White
            else
                colorData(i,j) = 0; % Black
            end
        end
    end

    % 缩放并平移球体
    sx = sx * r + com(1);
    sy = sy * r + com(2);
    sz = sz * r + com(3);

    % 绘制球体
    surf(sx, sy, sz, colorData, ...
         'EdgeColor', 'none', ...
         'FaceColor', 'flat');
    colormap(gca, gray); % 设置色图为黑白

    % --- 6. (可选) 绘制主惯量轴 ---
    if opts.ShowAxes
        scale = max(box_dims) * 0.6;
        quiver3(com(1), com(2), com(3), R_rot(1,1), R_rot(2,1), R_rot(3,1), scale, 'r', 'LineWidth', 2);
        quiver3(com(1), com(2), com(3), R_rot(1,2), R_rot(2,2), R_rot(3,2), scale, 'g', 'LineWidth', 2);
        quiver3(com(1), com(2), com(3), R_rot(1,3), R_rot(2,3), R_rot(3,3), scale, 'b', 'LineWidth', 2);
    end
    
    title('Inertial Visualization (Mass, CoM, Inertia Box)');
    hold off;
end