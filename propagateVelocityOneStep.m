function propagated_velocity = propagateVelocityOneStep(velocity_old,quat_new,quat_old,a_new,a_old,dt)

g = [0;0;9.81];
rotate_mat_new = quatToRotMat(quat_new);
rotate_mat_old = quatToRotMat(quat_old);

% take care of the  transpose of the rotation matrix
dv = (rotate_mat_new'*a_new+rotate_mat_old'*a_old)/2;

% take care of the sign before gravity g
propagated_velocity = velocity_old + ( dv - g)*dt;

