classdef URDF_to_MDH_Converter
    % URDF_TO_MDH_CONVERTER 将 URDF 文件转换为 DH 参数
    %
    % 修复说明:
    % 1. 修复了 'joint.name' 不存在的错误 (在 process_joint_xml 中添加了 name 字段)。
    % 2. 类名与文件名保持一致。
    
    properties
        UrdfFile
        Links       % Map: name -> struct
        Joints      % Map: name -> struct
        RootNode
        OrderedLinks % Cell array of link names in BFS order
        Verbose = false
    end

    methods
        %% 构造函数
        function obj = URDF_to_MDH_Converter(urdf_path)
            obj.UrdfFile = urdf_path;
            obj.Links = containers.Map();
            obj.Joints = containers.Map();
        end

        function dh_table = generate(obj)
            % 主流程
            obj = obj.parse_urdf();
            obj = obj.calculate_tfs_in_world_frame();
            dh_table = obj.calculate_dh_params();
        end
        
        %% 1. 解析 URDF
        function obj = parse_urdf(obj)
            try
                dom = xmlread(obj.UrdfFile);
            catch
                error('无法读取 URDF 文件: %s', obj.UrdfFile);
            end
            
            root = dom.getDocumentElement();
            
            % 提取 Links
            link_nodes = root.getElementsByTagName('link');
            for i = 0:link_nodes.getLength()-1
                item = link_nodes.item(i);
                name = char(item.getAttribute('name'));
                
                link_data = struct();
                link_data.name = name;
                link_data.rel_tf = eye(4);
                link_data.abs_tf = eye(4);
                link_data.dh_tf = eye(4);
                link_data.abs_dh_tf = eye(4);
                link_data.dh_found = false;
                link_data.parent_joint = '';
                link_data.children_joints = {};
                
                obj.Links(name) = link_data;
            end
            
            % 提取 Joints
            joint_nodes = root.getElementsByTagName('joint');
            for i = 0:joint_nodes.getLength()-1
                item = joint_nodes.item(i);
                [name, data] = obj.process_joint_xml(item);
                obj.Joints(name) = data;
                
                % 建立树结构关系
                if obj.Links.isKey(data.parent) && obj.Links.isKey(data.child)
                    child_link = obj.Links(data.child);
                    child_link.parent_joint = name;
                    obj.Links(data.child) = child_link;
                    
                    parent_link = obj.Links(data.parent);
                    parent_link.children_joints{end+1} = name;
                    obj.Links(data.parent) = parent_link;
                end
            end
            
            % 寻找根节点 (没有父 Joint 的 Link)
            link_names = obj.Links.keys;
            obj.RootNode = '';
            for i = 1:length(link_names)
                l = obj.Links(link_names{i});
                if isempty(l.parent_joint)
                    obj.RootNode = l.name;
                    break;
                end
            end
            
            if isempty(obj.RootNode)
                error('未找到根节点 Link');
            end
            
            % BFS 排序以确保计算顺序正确
            obj.OrderedLinks = {};
            queue = {obj.RootNode};
            while ~isempty(queue)
                current_link_name = queue{1};
                queue(1) = [];
                obj.OrderedLinks{end+1} = current_link_name;
                
                current_link = obj.Links(current_link_name);
                for j = 1:length(current_link.children_joints)
                    joint_name = current_link.children_joints{j};
                    joint = obj.Joints(joint_name);
                    queue{end+1} = joint.child;
                end
            end
            
            % 根节点的 DH 设为找到 (Identity)
            root_l = obj.Links(obj.RootNode);
            root_l.dh_found = true;
            obj.Links(obj.RootNode) = root_l;
        end

        %% 2. 计算世界坐标系下的变换
        function obj = calculate_tfs_in_world_frame(obj)
            for i = 1:length(obj.OrderedLinks)
                link_name = obj.OrderedLinks{i};
                if strcmp(link_name, obj.RootNode)
                    continue; 
                end
                
                link = obj.Links(link_name);
                joint = obj.Joints(link.parent_joint);
                parent_link = obj.Links(joint.parent);
                
                % 计算相对变换
                tf = eye(4);
                tf(1:3, 1:3) = obj.get_extrinsic_rotation(joint.rpy);
                tf(1:3, 4) = joint.xyz;
                
                link.rel_tf = tf;
                link.abs_tf = parent_link.abs_tf * tf;
                
                obj.Links(link_name) = link;
            end
        end

        %% 3. 计算 DH 参数 (Standard DH)
        function dh_table = calculate_dh_params(obj)
            results = {};
            
            for i = 1:length(obj.OrderedLinks)
                link_name = obj.OrderedLinks{i};
                link = obj.Links(link_name);
                
                if link.dh_found, continue; end
                
                % 当前 Link 到 World
                link_to_world = link.abs_tf;
                
                % 父 Link 的父 Link (Grandparent)
                joint = obj.Joints(link.parent_joint);
                parent_link = obj.Links(joint.parent);
                
                % 获取父 Link 的上一级 DH 变换
                parent_to_world_dh = parent_link.abs_dh_tf;
                
                % 计算从当前 Link 到父 DH 帧的变换
                link_to_parent_dh = inv(parent_to_world_dh) * link_to_world;
                
                % 获取轴向 (在 Link Frame 转换到 Parent DH Frame)
                axis_vec = link_to_parent_dh(1:3, 1:3) * joint.axis(:);
                
                dh_params = obj.get_joint_dh_params(link_to_parent_dh, axis_vec);
                
                % 更新 DH Frame
                dh_frame_mat = obj.get_dh_frame(dh_params);
                abs_dh_frame = parent_to_world_dh * dh_frame_mat;
                
                link.dh_tf = dh_frame_mat;
                link.abs_dh_tf = abs_dh_frame;
                link.dh_found = true;
                obj.Links(link_name) = link;
                
                % 构造结果行
                % 注意：这里直接使用 joint.name, 因为现在 process_joint_xml 已经包含了 name 字段
                row = struct(...
                    'Joint', joint.name, ...
                    'Parent', joint.parent, ...
                    'Child', joint.child, ...
                    'd', round(dh_params(1), 6), ...
                    'theta_rad', dh_params(2), ...
                    'a', round(dh_params(3), 6), ...
                    'alpha_rad', dh_params(4), ...
                    'theta_deg', round(rad2deg(dh_params(2)), 5), ...
                    'alpha_deg', round(rad2deg(dh_params(4)), 5) ...
                );
                results{end+1} = row;
            end
            
            if isempty(results)
                dh_table = table(); 
                warning('未生成 DH 参数，可能是因为只有一个 Link');
            else
                dh_table = struct2table([results{:}]);
            end
        end

        %% 核心几何计算逻辑
        function params = get_joint_dh_params(obj, rel_link_frame, axis_vec)
            % params: [d, theta, r, alpha]
            params = zeros(1, 4);
            origin_xyz = rel_link_frame(1:3, 4);
            z_axis = [0; 0; 1];
            
            is_collinear = obj.are_collinear([0;0;0], z_axis, origin_xyz, axis_vec);
            is_parallel = obj.are_parallel(z_axis, axis_vec);
            [is_intersect, ~] = obj.lines_intersect([0;0;0], z_axis, origin_xyz, axis_vec);
            
            if is_collinear
                params(1) = origin_xyz(3); % d
            elseif is_parallel
                params(1) = origin_xyz(3); % d
                params(2) = atan2(origin_xyz(2), origin_xyz(1)); % theta
                params(3) = sqrt(origin_xyz(1)^2 + origin_xyz(2)^2); % r (a)
            elseif is_intersect
                [~, x] = obj.lines_intersect([0;0;0], z_axis, origin_xyz, axis_vec);
                params(1) = x(1); % d
                
                % Intersect logic from python
                for k=1:3, if abs(axis_vec(k)) < 1e-5, axis_vec(k)=0; end; end
                
                cn = cross(z_axis, axis_vec);
                for k=1:3, if abs(cn(k)) < 1e-6, cn(k)=0; end; end
                
                if cn(1) < 0, cn = -cn; end
                
                params(2) = atan2(cn(2), cn(1)); % theta
                params(3) = 0.0; % r (a)
                
                vn = cn / norm(cn);
                val1 = dot(cross(z_axis, axis_vec), vn);
                val2 = dot(z_axis, axis_vec);
                params(4) = atan2(val1, val2); % alpha
            else
                % Skew case
                params = obj.process_skew_case(origin_xyz, axis_vec);
            end
        end

        function params = process_skew_case(~, origin, direction)
            pointA = [0; 0; 0];
            params = zeros(1, 4);
            
            denom = direction(1)^2 + direction(2)^2;
            if abs(denom) < 1e-9
                t = 0; 
            else
                t = -(origin(1)*direction(1) + origin(2)*direction(2)) / denom;
            end
            
            pointB = origin + t * direction;
            pointA(3) = pointB(3); 
            
            params(1) = pointA(3); % d
            params(3) = norm(pointB - pointA); % r (a)
            params(2) = atan2(pointB(2), pointB(1)); % theta
            
            cn = pointB - pointA;
            vn = cn / norm(cn);
            
            zaxis = [0;0;1];
            val1 = dot(cross(zaxis, direction), vn);
            val2 = dot(zaxis, direction);
            params(4) = atan2(val1, val2); % alpha
        end

        %% 辅助几何函数
        function res = are_parallel(~, v1, v2)
            v1 = v1 / norm(v1);
            v2 = v2 / norm(v2);
            res = all(abs(cross(v1, v2)) < 1e-5);
        end

        function res = are_collinear(obj, p1, v1, p2, v2)
            if ~obj.are_parallel(v1, v2), res = false; return; end
            if all(abs(p1 - p2) < 1e-5), res = true; return; end
            
            vec_diff = p2 - p1;
            if norm(vec_diff) < 1e-5, res=true; return; end
            res = obj.are_parallel(vec_diff, v1);
        end

        function [res, x] = lines_intersect(obj, p1, v1, p2, v2)
            x = zeros(2,1);
            if obj.are_collinear(p1, v1, p2, v2), res=false; return; end
            if obj.are_parallel(v1, v2), res=false; return; end
            
            A = [v1, -v2];
            b = p2 - p1;
            x = pinv(A) * b;
            
            point_on_1 = p1 + x(1)*v1;
            point_on_2 = p2 + x(2)*v2;
            
            dist = norm(point_on_1 - point_on_2);
            res = dist < 1e-5;
        end
        
        %% 辅助运动学函数
        function R = get_extrinsic_rotation(~, rpy)
            ct = cos(rpy(1)); st = sin(rpy(1)); Rx = [1,0,0; 0,ct,-st; 0,st,ct];
            ct = cos(rpy(2)); st = sin(rpy(2)); Ry = [ct,0,st; 0,1,0; -st,0,ct];
            ct = cos(rpy(3)); st = sin(rpy(3)); Rz = [ct,-st,0; st,ct,0; 0,0,1];
            R = Rz * Ry * Rx;
        end
        
        function T = get_dh_frame(~, params)
            d = params(1); theta = params(2);
            a = params(3); alpha = params(4);
            
            ct = cos(theta); st = sin(theta);
            ca = cos(alpha); sa = sin(alpha);
            
            % Standard DH Matrix
            T = [ct, -st*ca,  st*sa,  a*ct;
                 st,  ct*ca, -ct*sa,  a*st;
                 0,   sa,     ca,     d;
                 0,   0,      0,      1];
        end

        function [name, data] = process_joint_xml(~, joint_item)
            name = char(joint_item.getAttribute('name'));
            data = struct();
            % --- 修复点：添加了 name 字段 ---
            data.name = name; 
            data.type = char(joint_item.getAttribute('type'));
            data.axis = [1; 0; 0];
            data.xyz = [0; 0; 0];
            data.rpy = [0; 0; 0];
            data.parent = '';
            data.child = '';
            
            children = joint_item.getChildNodes();
            for j = 0:children.getLength()-1
                node = children.item(j);
                nodeName = char(node.getNodeName());
                if strcmp(nodeName, 'axis')
                    vals = str2num(node.getAttribute('xyz')); %#ok<ST2NM>
                    if ~isempty(vals), data.axis = vals(:); end
                elseif strcmp(nodeName, 'origin')
                    xyz = str2num(node.getAttribute('xyz')); %#ok<ST2NM>
                    rpy = str2num(node.getAttribute('rpy')); %#ok<ST2NM>
                    if ~isempty(xyz), data.xyz = xyz(:); end
                    if ~isempty(rpy), data.rpy = rpy(:); end
                elseif strcmp(nodeName, 'parent')
                    data.parent = char(node.getAttribute('link'));
                elseif strcmp(nodeName, 'child')
                    data.child = char(node.getAttribute('link'));
                end
            end
        end
    end
end