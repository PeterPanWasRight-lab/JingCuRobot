function [T_global, T_relative] = GetAllTransforms(Slist, Mlist, theta)
% GetAllLayerTransforms 计算机械臂所有中间连杆的坐标系变换矩阵
% 基于 Modern Robotics Space Frame PoE 公式
%
% 输入:
%   Slist: 6xN 螺旋轴矩阵 (Space Frame)，每一列是一个关节的螺旋轴
%   Mlist: 4x4x(N+1) 相对初始位姿矩阵列表 (M01, M12, ... M_{N,N+1})
%   theta: Nx1 关节角度向量
%
% 输出:
%   T_global:   4x4x(K) 矩阵，T_global(:,:,i) 表示 T_{0,i}
%               (从基座到第i个坐标系的变换)
%   T_relative: 4x4x(K) 矩阵，T_relative(:,:,i) 表示 T_{i-1, i}
%               (当前坐标系相对于上一级坐标系的变换)

    N = length(theta);          % 关节数量 (6)
    K = size(Mlist, 3);         % 坐标系变换数量 (7: M01...M67)
    
    % 初始化输出
    T_global = zeros(4, 4, K);
    T_relative = zeros(4, 4, K);
    
    % 累积变量初始化
    M_accum = eye(4);           % 累积的 Home Configuration (M0i)
    Exp_accum = eye(4);         % 累积的指数映射 (e^S1t1 * ... * e^Siti)
    T_prev = eye(4);            % 上一个全局变换 T_{0, i-1} (初始化为T00=I)
    
    % 循环计算每一个坐标系
    % 注意：通常坐标系 i 由关节 i 驱动。
    % 对于 UR5，有6个关节，但 Mlist 有7个 (M01...M67)，最后往往是法兰或工具
    
    for i = 1:K
        % 1. 获取当前层级的静态相对变换 M_{i-1, i}
        M_current_rel = Mlist(:, :, i);
        
        % 2. 计算当前关节的旋转 (仅当 i <= N 时存在关节运动)
        if i <= N
            % 使用 Modern Robotics 的 MatrixExp6
            % construct local exponential
            Exp_i = MatrixExp6(VecTose3(Slist(:, i)) * theta(i));
            
            % 更新全局累积旋转项: Exp_accum = e^S1 * ... * e^Si
            Exp_accum = Exp_accum * Exp_i;
        end
        
        % 3. 更新全局 Home Pose: M0i = M01 * ... * M_{i-1,i}
        M_accum = M_accum * M_current_rel;
        
        % 4. 计算当前连杆的全局变换 T_{0,i}
        % Space PoE 公式: T0i = (e^S1t1...e^Siti) * M0i
        T_curr_global = Exp_accum * M_accum;
        T_global(:, :, i) = T_curr_global;
        
        % 5. 计算相对变换 T_{i-1, i}
        % T_{i-1, i} = inv(T_{0, i-1}) * T_{0, i}
        % 这种方法利用了矩阵逆，物理意义最清晰
        T_curr_rel = (T_prev \ T_curr_global); % 左除相当于 inv(A)*B，但在数值上更稳定
        T_relative(:, :, i) = T_curr_rel;
        
        % 更新上一次状态供下一次迭代使用
        T_prev = T_curr_global;
    end
end