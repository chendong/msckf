function synchronoused_index = synchronousTimeStamp(accel_timestamp,gyro_timestamp,tracks_timestamp)


%brute way
N = size(tracks_timestamp,1);
synchronoused_index = zeros(3,N);
for tracks_index = 1:N
    track_t = tracks_timestamp(tracks_index);
    
    accel_abs_diff = abs(accel_timestamp - track_t);
    accel_index = find(accel_abs_diff == min(accel_abs_diff));
    
    gyro_abs_diff = abs(gyro_timestamp - track_t);
    gyro_index = find(gyro_abs_diff == min(gyro_abs_diff));
    
    synchronoused_index(1,tracks_index) = tracks_index;
    synchronoused_index(2,tracks_index) = accel_index;
    synchronoused_index(3,tracks_index) = gyro_index;
    
    
end



end
