[TimeSeq,Gyro_m,Accl_m]=ExtractIMUInfo(imu);

% alloc space
imuStates = cell(1,numel(TimeSeq));
imuStates{1}.q_IG = [0,0,0,1]';         %4x1 Global to IMU rotation quaternion
imuStates{1}.v_IG = [0,0,0]';       %3x1 IMU Velocity in the Global Frmae
imuStates{1}.p_IG = [0,0,0]';      %3x1 IMU Position in the Global frame
imuStates{1}.b_g = [0,0,0]';       %3x1 Gyro bias
imuStates{1}.b_v = [0,0,0]';        %3x1 Velocity bias

% gyro bias
bw =[-0.003;
0.021;
0.079];

% accl bias
ba =[-0.024;
0.137;
0.076];

gravity =[0;
    0;
    9.81];
Sample_num = size(TimeSeq);
for itr = 2:Sample_num
    dt = (TimeSeq(itr)-TimeSeq(itr-1))/10e9;
    
    last_gyro_m = Gyro_m(:,itr-1);
    last_accl_m = Accl_m(:,itr-1);
    
    cur_gyro_m = Gyro_m(:,itr);
    cur_accl_m = Accl_m(:,itr);
    
    ewold = last_gyro_m - bw;
    eaold = last_accl_m - ba;
    
    ew = cur_gyro_m - bw;
    ea = cur_accl_m - ba;
    
    Omega = omegaMat(ew);
    Omegaold = omegaMat(ewold);
    OmegaMean = omegaMat((ew+ewold)/2);
    
    % first order quaternion integration
    div =1;
    MatExp = eye(4);
    OmegaMean = OmegaMean*0.5*dt;
    OmegaMean_I = OmegaMean;
    for i = 1:4
        div = div*i;
        MatExp = MatExp + OmegaMean_I/div;
        OmegaMean_I = OmegaMean_I*OmegaMean; 
    end
    quat_int = MatExp + 1.0 / 48.0 * (Omega * Omegaold - Omegaold * Omega) * dt * dt;
    
    quat_propagated = quat_int*imuStates{itr-1}.q_IG;   % ”–Œ Ã‚
    %imuStates{itr}.q_IG = quatNormalize(quat_propagated);
    imuStates{itr}.q_IG = quatnormalize(quat_propagated')';
    
    % propagating velocity and position 
    last_RotMat = quatToRotMat(imuStates{itr-1}.q_IG);
    cur_RotMat = quatToRotMat(imuStates{itr}.q_IG);
    dv = (cur_RotMat*ea + last_RotMat*eaold)/2;
    imuStates{itr}.v_IG = imuStates{itr-1}.v_IG + (dv - gravity)*dt;
    imuStates{itr}.p_IG = imuStates{itr-1}.p_IG + (imuStates{itr}.v_IG +imuStates{itr-1}.v_IG)/2*dt;
 
end
Position = zeros(3,Sample_num(1));
for i = 1:Sample_num
    Position(:,i) = imuStates{i}.p_IG;
    
end

plot3(Position(1,:),Position(2,:),Position(3,:));




