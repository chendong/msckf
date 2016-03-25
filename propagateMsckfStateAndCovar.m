function msckfState_prop = propagateMsckfStateAndCovar(msckfState, measurements_k, noiseParams)

    
    % cam1-----------------------cam2
    %   |--------------------------|
    %   +---+---+---+---+---+---+--+
    %  i1   i2  i3  i4  i5  i6  i7 i8
    
    
    imu_reading_num = size(measurements_k.imu_reading,1);
    imu_reading_timestamp = measurements_k.imu_reading(:,1);
    gyro_reading = measurements_k.imu_reading(:,2:4);
    accel_reading = measurements_k.imu_reading(:,5:7);
    
    % 
    for imu_integrating_itr = 2:imu_reading_num
        % get imu reading current now
        dt = imu_reading_timestamp(imu_integrating_itr) - imu_reading_timestamp(imu_integrating_itr-1);
        
        % Jacobians
        Q_imu = noiseParams.Q_imu;
        
        % using current imu reading to calculate F and G
        cur_gyro_reading = gyro_reading(imu_integrating_itr)';
        cur_accel_reading  = accel_reading(imu_integrating_itr)';
        F = calcF(msckfState.imuState,cur_gyro_reading,cur_accel_reading);
        G = calcG(msckfState.imuState);
        
        %Propagate State
        last_gyro_reading = gyro_reading(imu_integrating_itr-1)';
        last_accel_reading  = accel_reading(imu_integrating_itr-1)';
        msckfState_new.imuState = propagateImuState(msckfState.imuState, cur_gyro_reading,cur_accel_reading,last_gyro_reading,last_accel_reading,dt);
        
        % State Transition Matrix
        Phi = eye(size(F,1)) + F * dt; % Leutenegger 2013
        
        % IMU-IMU Covariance
%     msckfState_prop.imuCovar = msckfState.imuCovar + ...
%                                 ( F * msckfState.imuCovar ...
%                                 + msckfState.imuCovar * F' ...
%                                 + G * Q_imu * G' ) ...
%                                         * measurements_k.dT;

        msckfState_new.imuCovar = Phi * msckfState.imuCovar * Phi' ...
                                + G * Q_imu * G' * dt; % Leutenegger 2013
        % Enforce PSD-ness  
        msckfState_new.imuCovar = enforcePSD(msckfState_new.imuCovar);
        
        % Camera-Camera Covariance
        msckfState_new.camCovar = msckfState.camCovar;
        
        % IMU-Camera Covariance
        msckfState_new.imuCamCovar = Phi * msckfState.imuCamCovar;
        msckfState_new.camStates = msckfState.camStates;
        
        msckfState = msckfState_new;
        
    end
    msckfState_prop = msckfState;
  
end