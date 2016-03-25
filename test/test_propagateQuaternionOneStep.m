function propagated_quaternion=test_propagateQuaternionOneStep(start_index,stop_index,quat,gyro_m,bg,imu_timestamp)
% test function propagateQuaternionOneStep

%add the first propagated quat using the groudtruth one
propagated_quaternion(:,start_index) = quat(:,start_index);

for i=start_index+1:stop_index
    dt = (imu_timestamp(i)-imu_timestamp(i-1))/1e9;
    propagated_quat = propagateQuaternionOneStep(propagated_quaternion(:,i-1),gyro_m(:,i-1)-bg(i-1),gyro_m(:,i)-bg(i),dt);
    propagated_quaternion(:,i) = propagated_quat;
    
end
end
