function interpolated_accel = interpolateAccelData(acc_timestamp,gyro_timestamp,accel)

% get the gyro alinged index with accel data
gyro_aligned_index  = synchronousAccelGyroTimeStamp(acc_timestamp,gyro_timestamp);
n=size(gyro_aligned_index,2);
max_index_accel = size(acc_timestamp);
% alloc
interpolated_accel = zeros(4,n);
for i = 1:n
    % get the 
    gyro_aligned_accel_index = gyro_aligned_index(2,i);
    accel_aligned_t = acc_timestamp(gyro_aligned_accel_index);
    accel_elem = accel(gyro_aligned_accel_index,2:4);
    gyro_t = gyro_timestamp(i);
    % corner case
   % if (gyro_aligned_accel_index==1 && gyro_t < accel_aligned_t) || (gyro_aligned_accel_index == max_index_accel && gyro_t >  accel_aligned_t)
   if (gyro_aligned_accel_index==1  ) 
       disp('first element cornor case ')
        interpolated_accel(1,i) = i;
        interpolated_accel(2:4,i) = accel_elem';
   elseif gyro_aligned_accel_index == max_index_accel 
       disp('last element cornor case');
        interpolated_accel(1,i) = i;
        
        interpolated_accel(2:4,i) = accel_elem';
       
   else
      
         
        % if the closest 
        if gyro_t < accel_aligned_t
            % find the accel previous timestamp
            accel_aligned_pre_t = acc_timestamp(gyro_aligned_accel_index-1);
            % find the accel previous value
            accel_elem_pre = accel(gyro_aligned_accel_index-1,2:4);

            interpolate_accel = interpolate(gyro_t,accel_aligned_pre_t,accel_aligned_t,accel_elem_pre,accel_elem);
            interpolated_accel(1,i) = i;
            interpolated_accel(2:4,i) = interpolate_accel';

        else
            % find the next accel timestamp and value
            accel_aligned_next_t = acc_timestamp(gyro_aligned_accel_index+1);
            accel_elem_next = accel(gyro_aligned_accel_index+1,2:4);

            interpolate_accel = interpolate(gyro_t,accel_aligned_t,accel_aligned_next_t,accel_elem,accel_elem_next);
            interpolated_accel(1,i) = i;
            interpolated_accel(2:4,i) = interpolate_accel';
        end
    end

end



end