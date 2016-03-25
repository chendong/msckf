function F = calcF(imuState_k, currentGyroReading,currentAccelReading)
% Multiplies the error state in the linearized continuous-time
% error state model
% currentGyroReading: 3*1 vector
% currentAccelReading: 3*1 vector
% add velocity component


    F = zeros(15,15);
    
    omegaHat = currentGyroReading - imuState_k.b_g;
    aHat = currentAccelReading - imuState_k.b_a;
    C_IG = quatToRotMat(imuState_k.q_IG);
    
    F(1:3,1:3) = -crossMat(omegaHat);
    F(1:3,4:6) = -eye(3);
    F(10:12,1:3) = -C_IG' * crossMat(aHat);
    F(10:12,7:9) = -C_IG';
    F(13:15,10:12) = eye(3);
  
end