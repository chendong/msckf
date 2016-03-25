function imuState_prop = propagateImuState(imuState_k, cur_gyro_reading,cur_accel_reading,last_gyro_reading,last_accel_reading,dt)
% prop == propagated to k+1
    
    cur_omegaHat = cur_gyro_reading - imuState_k.b_g;
    last_omegaHat = last_gyro_reading - imuState_k.b_g;
    
    cur_aHat = cur_accel_reading - imuState_k.b_a;
    last_aHat = last_accel_reading - imuState_k.b_a;
    % C_IG = quatToRotMat(imuState_k.q_IG);
   
    old_quat = imuState_k.q_IG;
    old_v = imuState_k.v_I_G;
    old_p    = imuState_k.p_I_G;
 
    % Rotation state
    new_quat = propagateQuaternionOneStep(old_quat,cur_omegaHat,last_omegaHat,dt);  
    %Unit length quaternion
    new_quat = new_quat/norm(new_quat);
    
    % velocity state
    new_v = propagateVelocityOneStep(old_v,new_quat,old_quat,cur_aHat,last_aHat,dt);
    
    % position state 
    new_p =propagatePositionOneStep(old_p,new_v,old_v,dt);
    
    % update q,v,p
    imuState_prop.q_IG = new_quat;
    imuState_prop.v_I_G = new_v;
    imuState_prop.p_I_G = new_p;

    % Bias states
    imuState_prop.b_g = imuState_k.b_g;
    imuState_prop.b_a= imuState_k.b_a;

end