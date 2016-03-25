clear;
close all;
clc;
addpath('../utils');

% INPUT1: raw imu data
imudatafile = '../dataset/EuRoC_MAV_Dataset/MH_01_easy/groud_aligned_imu.mat';
load(imudatafile);
% INPUT2: groud truth data
groudtruthdatafile = '../dataset/EuRoC_MAV_Dataset/MH_01_easy/state_groundtruth_estimate0.mat';
load(groudtruthdatafile);
imu_reading  = groud_aligned_imu;
groud_truth = state_groudtruth_estimate;
kStart = 3000; kEnd = 5000;

sample_num = size(imu_reading,1);
% alloc
imuStates = cell(1,sample_num);
rotation_viz = zeros(4,sample_num);
position_viz = zeros(3,sample_num);

%% ==========================Initial State======================== %%
% fill the field of first imustate,using groud truth data
 firstImuState.q_IG = [groud_truth(kStart,6:8)';groud_truth(kStart,5)];
 firstImuState.q_IG =  firstImuState.q_IG/norm(firstImuState.q_IG);
 firstImuState.v_I_G = groud_truth(kStart,9:11)';
 firstImuState.p_I_G = groud_truth(kStart,2:4)';
 firstImuState.b_g = groud_truth(kStart,12:14)';
 firstImuState.b_a = groud_truth(kStart,15:17)';
 
 imuStates{kStart} = firstImuState;
 rotation_viz(:,kStart) = firstImuState.q_IG;
 position_viz(:,kStart) = firstImuState.p_I_G;
 %% ============================MAIN LOOP========================== %%

for state_k = kStart:(kEnd-1)
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

sub_sample_num = kEnd - kStart+1;
p = zeros(3,sample_num);
q = zeros(4,sample_num);
q(1,:) = 1;
for i = kStart:kEnd
    p(:,i)=position_viz(:,i)';
    q(:,i)=[rotation_viz(4,i);rotation_viz(1:3,i)];    
end

figure(1);
q_plotPose(p(:, kStart), q(:, kStart), '', 0.5);
hold on;
for i = kStart+1:SubsampleFactor:kEnd
    q_plotPose(p(:, i), q(:, i), '', 0.05);
end
plot3(p(1, kStart:kEnd), p(2, kStart:kEnd), p(3,kStart:kEnd), 'k');

figure(2);
plot_subRejectory(state_groudtruth_estimate( kStart:kEnd,:),100);
 
 


