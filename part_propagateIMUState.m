%part_propagateIMUstate
% this is used to verify the propagation part
clear;
close all;
clc;
addpath('utils')

fileName = './dataset/tango/data1/dataset_camera_alignedindex_featuretracks.mat'; 
load(fileName);
% camera,gyro,accel aligned index
aligned_index = syn_index;
% gyro,accel aligned data
aligned_imu_reading = aligned_gyro_accel;
sample_num = size(aligned_imu_reading,1);
imuStates = cell(1,sample_num);
rotation_viz = zeros(4,sample_num);
position_viz = zeros(3,sample_num);

% sub dataset
camStart = 2200; camEnd = 2300;
imuStart = aligned_index(3,camStart);
imuEnd = aligned_index(3,camEnd);


%% ==========================Initial State======================== %%
% fill the field of first imustate,using groud truth data
 firstImuState.q_IG = [0;0;0;1];
 firstImuState.q_IG = firstImuState.q_IG/norm(firstImuState.q_IG);
 firstImuState.v_I_G = [0;0;0];
 firstImuState.p_I_G =[0;0;0];
 firstImuState.b_g = [0;0;0];
 firstImuState.b_a = [0;0;0];
 
 imuStates{imuStart} = firstImuState;
 rotation_viz(:,imuStart) = firstImuState.q_IG;
 position_viz(:,imuStart) = firstImuState.p_I_G;
 
  %% ============================MAIN LOOP========================== %%

for imustate_k = imuStart:(imuEnd-1)
    fprintf('imustate_k = %4d\n', imustate_k);
    
    %% ==========================STATE PROPAGATION======================== %%
    
    dt = (aligned_imu_reading(imustate_k+1,1) - aligned_imu_reading(imustate_k,1));
    % propagate rotation
    old_omegaHat = aligned_imu_reading(imustate_k,2:4)' - imuStates{imustate_k}.b_g;
    new_omegaHat = aligned_imu_reading(imustate_k+1,2:4)' - imuStates{imustate_k}.b_g;
    propagated.q_IG = propagateQuaternionOneStep(imuStates{imustate_k}.q_IG,new_omegaHat,old_omegaHat,dt);
    propagated.q_IG = propagated.q_IG/norm(propagated.q_IG);
    
    % propagate velocity
    old_aHat = aligned_imu_reading(imustate_k,5:7)' - imuStates{imustate_k}.b_a;
    new_aHat = aligned_imu_reading(imustate_k+1,5:7)' - imuStates{imustate_k}.b_a;
    propagated.v_I_G = propagateVelocityOneStep(imuStates{imustate_k}.v_I_G,propagated.q_IG,imuStates{imustate_k}.q_IG,new_aHat,old_aHat,dt);
    
    
    % propagate position
    propagated.p_I_G=propagatePositionOneStep(imuStates{imustate_k}.p_I_G,propagated.v_I_G,imuStates{imustate_k}.v_I_G,dt);
    % propagate bias
    propagated.b_g = imuStates{imustate_k}.b_g;
    propagated.b_a = imuStates{imustate_k}.b_a;
    
    imuStates{imustate_k+1} = propagated;
    rotation_viz(:,imustate_k+1) = propagated.q_IG;
    position_viz(:,imustate_k+1) = propagated.p_I_G;
    
    
end


%% ==========================PLOT RESULT======================== %%
SubsampleFactor = 100;

sub_sample_num = imuEnd - imuStart+1;
p = zeros(3,sample_num);
q = zeros(4,sample_num);
q(1,:) = 1;
for i = imuStart:imuEnd
    p(:,i)=position_viz(:,i)';
    q(:,i)=[rotation_viz(4,i);rotation_viz(1:3,i)];    
end

figure(1);
q_plotPose(p(:, imuStart), q(:, imuStart), '', 0.5);
hold on;
for i = imuStart+1:SubsampleFactor:imuEnd
    q_plotPose(p(:, i), q(:, i), '', 0.05);
end
plot3(p(1, imuStart:imuEnd), p(2, imuStart:imuEnd), p(3,imuStart:imuEnd), 'k');

% figure(2);
% plot_subRejectory(state_groudtruth_estimate( kStart:kEnd,:),100);
 