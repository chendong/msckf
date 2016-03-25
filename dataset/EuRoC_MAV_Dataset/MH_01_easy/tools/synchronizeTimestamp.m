function syn_index = synchronizeTimestamp(image_timestamp,raw_imu_timestamp,groudtruth_timestamp)
% synchronize timestamp at different Frequency Range
% return the synchronized index
image_num = size(image_timestamp,1);
%syn_index = zeros(3:image_num);
for i = 1:image_num
    syn_index(1,i) = i;
    img_t = image_timestamp(i);
    diffabs_raw_imu = abs(raw_imu_timestamp - img_t);
    raw_imu_index = find(diffabs_raw_imu == min(diffabs_raw_imu));
    syn_index(2,i) = raw_imu_index;
    
    diffabs_groud = abs(groudtruth_timestamp - img_t);
    groud_index = find(diffabs_groud == min(diffabs_groud));
    
    syn_index(3,i) = groud_index;
    
    
end
end
