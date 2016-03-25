function propagated_position=propagatePositionOneStep(position_old,velocity_new,velocity_old,dt)
%
propagated_position = position_old+(velocity_new+velocity_old)/2*dt;
end