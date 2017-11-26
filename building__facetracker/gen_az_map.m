maskMap = containers.Map('KeyType', 'int32', 'ValueType', 'any');
for sampled_azimuth = -50:1:50
   mask = double(imread(strcat('mesh_test/2mask_im_az',num2str(sampled_azimuth),'.png')))./(255.0);
   maskMap(sampled_azimuth) = mask;
end