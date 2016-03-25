function gyro_aligned_index= synchronousAccelGyroTimeStamp(acc_timestamp,gyro_timestamp)

n=size(gyro_timestamp,1);
gyro_aligned_index = zeros(2,n);
size_accel = size(acc_timestamp);
for i = 1:n
   
    abs_diff = abs(acc_timestamp - gyro_timestamp(i));
    
    syn_acc_index = find(abs_diff == min (abs_diff));
    
    syn_acc_index =  syn_acc_index(1);
    
    if  syn_acc_index>size_accel
        error('error in find index');
    end
    
    gyro_aligned_index(1,i) = i;
    % syn_acc_index may have multiple value
    gyro_aligned_index(2,i) = syn_acc_index;
    
    
end

end