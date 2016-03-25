clear;
close all;
clc;
addpath('../utils');

% INPUT: data include:
% 1:syn_index
% 2:state_groudtruth_estimate
% 3.image_timestamp
% 4.groud_aligned_imu

datafile = '../dataset/EuRoC_MAV_Dataset/MH_01_easy/MH_01_easy.mat';camStart = 100; camEnd = 1000;
load(datafile);

aligned_index = syn_index;
groud_truth = state_groudtruth_estimate;
imu_reading = groud_aligned_imu;

imuStart = syn_index(2,camStart);imuEnd = syn_index(2,camEnd);
imu_sample_num = size(imu_reading,1);

% alloc
imuStates = cell(1,imu_sample_num);
rotation_viz = zeros(4,imu_sample_num);
position_viz = zeros(3,imu_sample_num);

%% ==========================Initial State======================== %%
% fill the field of first imustate,using groud truth data
 firstImuState.q_IG = [groud_truth(imuStart,6:8)';groud_truth(imuStart,5)];
 firstImuState.q_IG =  firstImuState.q_IG/norm(firstImuState.q_IG);
 firstImuState.v_I_G = groud_truth(imuStart,9:11)';
 firstImuState.p_I_G = groud_truth(imuStart,2:4)';
 firstImuState.b_g = groud_truth(imuStart,12:14)';
 firstImuState.b_a = groud_truth(imuStart,15:17)';
 
 imuStates{imuStart} = firstImuState;
 rotation_viz(:,imuStart) = firstImuState.q_IG;
 position_viz(:,imuStart) = firstImuState.p_I_G;
 %% ============================MAIN LOOP========================== %%


for state_k = imuStart:(imuEnd-1)
    fprintf('state_k = %4d\n', state_k);
    
    %% ==========================STATE PROPAGATION======================== %%
    
    dt = (imu_reading(state_k+1,1) - imu_reading(state_k,1))/1e9;
    % propagate rotation
    old_omegaHat = imu_reading(state_k,2:4)' - imuStates{state_k}.b_g;
    new_omegaHat = imu_reading(state_k+1,2:4)' - imuStates{state_k}.b_g;
    propagated.q_IG = propagateQuaternionOneStep(imuStates{state_k}.q_IG,new_omegaHat,old_omegaHat,dt);
    propagated.q_IG = propagated.q_IG/norm(propagated.q_IG);
    
    % propagate velocity
    old_aHat = imu_reading(state_k,5:7)' - imuStates{state_k}.b_a;
    new_aHat = imu_reading(state_k+1,5:7)' - imuStates{state_k}.b_a;
    propagated.v_I_G = propagateVelocityOneStep(imuStates{state_k}.v_I_G,propagated.q_IG,imuStates{state_k}.q_IG,new_aHat,old_aHat,dt);
    
    
    % propagate position
    propagated.p_I_G=propagatePositionOneStep(imuStates{state_k}.p_I_G,propagated.v_I_G,imuStates{state_k}.v_I_G,dt);
    % propagate bias
    
    propagated.b_g = groud_truth(state_k+1,12:14)';
    propagated.b_a = groud_truth(state_k+1,15:17)';
    
    imuStates{state_k+1} = propagated;
    rotation_viz(:,state_k+1) = propagated.q_IG;
    position_viz(:,state_k+1) = propagated.p_I_G;
    
    
end
%% ==========================PLOT RESULT======================== %%



SubsampleFactor = 100;

sub_sample_num = imuEnd - imuStart+1;
p = zeros(3,imu_sample_num);
q = zeros(4,imu_sample_num);
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

figure(2);
plot_subRejectory(state_groudtruth_estimate( imuStart:imuEnd,:),100);
 
 


