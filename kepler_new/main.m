% 实例化转换器
converter = URDF_to_MDH_Converter('left_arm.urdf');

% 生成 DH 参数表
dh_table = converter.generate();

% 显示结果
disp(dh_table);