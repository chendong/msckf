function [imu_timestamp,gyro_m,accl_m,groudTruth_timestamp,position,quat,velocity,bg,ba] = ExtractData(groudtruth_aligned_imu,groud_truth)

%ExtractData
%extract kinds of data from:
% 1) groudtruth_aligned_imu
% 2) groud_truth

%timestamp [ns]
%  using [0;0;0;1] as quaternion convention.

imu_timestamp = groudtruth_aligned_imu(:,1)';
gyro_m = groudtruth_aligned_imu(:,2:4)';
accl_m = groudtruth_aligned_imu(:,5:7)';

groudTruth_timestamp = groud_truth(:,1)';
position = groud_truth(:,2:4)';

% [0;0;0;1]
quat(4,:) = groud_truth(:,5)';
quat(1:3,:)=groud_truth(:,6:8)';

velocity = groud_truth(:,9:11)';
bg = groud_truth(:,12:14)';
ba = groud_truth(:,15:17)';




end