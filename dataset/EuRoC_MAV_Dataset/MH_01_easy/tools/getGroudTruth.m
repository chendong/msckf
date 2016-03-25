function [theta_vk_i,r_i_vk_i,t,w_vk_vk_i,v_vk_vk_i] = getGroudTruth(syn_index,image_timestamp,imu_reading,groudtruth)
image_sample = size(image_timestamp,1);
%theta_vk_i = zeros(3,image_sample);  % theta_vk_i is a Axis¨Cangle representation
%r_i_vk_i = zeros(3,image_sample);
%t = zeros(1,image_sample);
%w_vk_vk_i = zeros(3:image_sample);
%v_vk_vk_i = zeros(3,image_sample);
for i = 1:image_sample
    corresponding_index_in_imureading = syn_index(2,i);
    corresponding_index_in_groudtruth = syn_index(3,i);
    
    % theta, convert quaternion to axis-angle presentation
    quat = groudtruth(corresponding_index_in_groudtruth,5:8);
    axang = quat2axang(quat);
    theta = axang(4);
    axis = axang(1:3)/norm(axang(1:3));
    
    theta_vk_i(:,i) = theta*axis';
    
    % r_i_vk_i
    r_i_vk_i(:,i) = groudtruth(corresponding_index_in_groudtruth,2:4)';
    
    % t
    t = (image_timestamp(:,1)/1e9)';
    
    %w_vk_vk_i   gyro_reading
    w_vk_vk_i(:,i) = (imu_reading(corresponding_index_in_imureading,2:4)-groudtruth(corresponding_index_in_groudtruth,12:14))';
    
    %v_vk_vk_i
    v_vk_vk_i(:,i) = groudtruth(corresponding_index_in_groudtruth,9:11)';
end

end