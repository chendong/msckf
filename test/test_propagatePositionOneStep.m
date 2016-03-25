function propagated_position = test_propagatePositionOneStep(start_index,stop_index,position,velocity,imu_timestamp)

% test function

propagated_position (:,1) = position(:,1);

for i=start_index+1:stop_index
    dt = (imu_timestamp(i)-imu_timestamp(i-1))/1e9;
    propagated_position(:,i)=propagatePositionOneStep(propagated_position(:,i-1),velocity(:,i-1),velocity(:,i),dt);
end
end