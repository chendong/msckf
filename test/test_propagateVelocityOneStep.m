function propagated_velocity =test_propagateVelocityOneStep(start_index,stop_index,quat,velocity,accl_m,ba,imu_timestamp)
% test propagateVelocityOneStep function
propagated_velocity(:,1) = velocity(:,1);

for i = start_index+1:stop_index
    dt = (imu_timestamp(i)-imu_timestamp(i-1))/1e9;
    propagated_vel = propagateVelocityOneStep(propagated_velocity(:,i-1),...
        quat(:,i-1),quat(:,i),accl_m(:,i-1)-ba(:,i-1),accl_m(:,i)-ba(:,i),dt);
    propagated_velocity(:,i) = propagated_vel;
end
end