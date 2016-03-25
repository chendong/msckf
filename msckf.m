clear;
close all;
clc;
addpath('utils');

tic
dataDir = './dataset/tango/data1';
fileName = './dataset/tango/data1/dataset_camera_alignedindex_featuretracks.mat'; kStart = 2099; kEnd = 2300;

load(fileName);


%Set up the camera parameters 
camera.c_u      = cu;                   % Principal point [u pixels] 
camera.c_v      = cv;                   % Principal point [v pixels]
camera.f_u      = fu;                   % Focal length [u pixels]
camera.f_v      = fv;                   % Focal length [v pixels]
camera.w        = w;                    % distortion
camera.q_CI     = rotMatToQuat(C_c_v);  % 4x1 IMU-to-Camera rotation quaternion
camera.p_C_I    = rho_v_c_v;            % 3x1 Camera position in IMU frame


%Set up the noise parameters
y_var = 11^2 * ones(1,4);               % pixel coord var  
noiseParams.u_var_prime = y_var(1)/camera.f_u^2;
noiseParams.v_var_prime = y_var(2)/camera.f_v^2;

w_var = 4e-2 * ones(1,3);               % rot vel var  
a_var = 4e-2 * ones(1,3);               % lin accel var  
dbg_var = 1e-6 * ones(1,3);            % gyro bias change var 
dba_var = 1e-6 * ones(1,3);            % accel bias change var
noiseParams.Q_imu = diag([w_var, dbg_var, a_var, dba_var]);

% state : [q bg ba v p]
q_var_init = 1e-6 * ones(1,3);         % init rot var
bg_var_init = 1e-6 * ones(1,3);        % init gyro bias var
ba_var_init = 1e-6 * ones(1,3);        % init accel bias var
v_var_init = 1e-6 * ones(1,3);         % init velocity var
p_var_init = 1e-6 * ones(1,3);         % init pos var
noiseParams.initialIMUCovar = diag([q_var_init, bg_var_init, ba_var_init,v_var_init, p_var_init]);
   
   
% MSCKF parameters
msckfParams.minTrackLength = 10;        % Set to inf to dead-reckon only
msckfParams.maxTrackLength = Inf;      % Set to inf to wait for features to go out of view
msckfParams.maxGNCostNorm  = 1e-2;     % Set to inf to allow any triangulation, no matter how bad
msckfParams.minRCOND       = 1e-12;
msckfParams.doNullSpaceTrick = true;
msckfParams.doQRdecomp = true;



%% ========================== Prepare Data======================== %%
% Important: Because we're idealizing our pixel measurements and the
% idealized measurements could legitimately be -1, replace our invalid
% measurement flag with NaN
prunedStates = {};
% IMU state for plotting etc. Structures indexed in a cell array
imuStates = cell(1,numel(tracks_timestamp));

% idealize feature
undistored_featuretracks = undistortFeatureTracks(featuretracks,cu,cv,fu,fv,w);
y_k_j = transformFeatureTracksFormat(undistored_featuretracks); % transform the tango format 
y_k_j(y_k_j == -1) = NaN;
measurements = cell(1,numel(tracks_timestamp));
numLandmarks = size(y_k_j,3);
% get camera-gyro aligned index
aligned_index = syn_index;
aligned_imu_reading = aligned_gyro_accel;



for state_k = kStart:kEnd
    % get IMU readings during the Period between previous and current image
    % coming 
    start_gyro_accel_index =  aligned_index(3,state_k);
    end_gyro_accel_index   =  aligned_index(3,state_k+1);
    
    measurements{state_k}.index        = state_k;
    measurements{state_k}.imu_reading  = aligned_imu_reading(start_gyro_accel_index:end_gyro_accel_index,:);
    measurements{state_k}.y            = squeeze(y_k_j(1:2,state_k,:));  
     
    
    % a for-loop comsume too much time
    % Idealize camera measurements  
%     for i = 1:numLandmarks
%         if  ~isnan(measurements{state_k}.y(1,i))
%             measurements{state_k}.y(1,i) = (measurements{state_k}.y(1,i) - camera.c_u)/camera.f_u;
%             measurements{state_k}.y(2,i) = (measurements{state_k}.y(2,i) - camera.c_v)/camera.f_v;
%             % undistortiuon
%             distR = sqrt(measurements{state_k}.y(1,i)*measurements{state_k}.y(1,i) + measurements{state_k}.y(2,i)*measurements{state_k}.y(2,i));
%             dOneOver2Tan = 1.0 / (2.0 * tan(camera.w / 2.0));
%             R = tan(distR * camera.w) * dOneOver2Tan;
%             
%             if distR > 0.01
%                 dFactor =  R / distR;
%             else
%                 dFactor = 1.0;
%             end
%             
%             measurements{state_k}.y(1,i) = measurements{state_k}.y(1,i)*dFactor;
%             measurements{state_k}.y(2,i) = measurements{state_k}.y(2,i)*dFactor;
%         end
%     end


end


%% ==========================Initial State======================== %%
% fill the field of first imustate
% firstImuState.q_IG = rotMatToQuat(axisAngleToRotMat(theta_vk_i(:,kStart)));
% firstImuState.p_I_G = r_i_vk_i(:,kStart);
% firstImuState.q_IG = rotMatToQuat(rotx(90));
 firstImuState.q_IG = [0;0;0;1];
 firstImuState.b_g = zeros(3,1);
 firstImuState.b_a = zeros(3,1);
 firstImuState.v_I_G = [0;0;0];
 firstImuState.p_I_G = [0;0;0];
 
% initialize the first state
[msckfState, featureTracks, trackedFeatureIds] = initializeMSCKF(firstImuState, measurements{kStart}, camera, kStart, noiseParams);
imuStates = updateStateHistory(imuStates, msckfState, camera, kStart);
msckfState_imuOnly{kStart} = msckfState;
%% ============================MAIN LOOP========================== %%

numFeatureTracksResidualized = 0;
map = [];

for state_k = kStart:(kEnd-1)
    fprintf('state_k = %4d\n', state_k);
    
    %% ==========================STATE PROPAGATION======================== %%
    
    %Propagate state and covariance
    msckfState = propagateMsckfStateAndCovar(msckfState, measurements{state_k}, noiseParams);
    msckfState_imuOnly{state_k+1} = propagateMsckfStateAndCovar(msckfState_imuOnly{state_k}, measurements{state_k}, noiseParams);
    %Add camera pose to msckfState
    msckfState = augmentState(msckfState, camera, state_k+1);
    
    
     %% ==========================FEATURE TRACKING======================== %%
    % Add observations to the feature tracks, or initialize a new one
    % If an observation is -1, add the track to featureTracksToResidualize
    featureTracksToResidualize = {};
    
    
    % 
    for featureId = 1:numLandmarks
        %IMPORTANT: state_k + 1 not state_k
        meas_k = measurements{state_k+1}.y(:, featureId);
        
        outOfView = isnan(meas_k(1,1));
        
        if ismember(featureId, trackedFeatureIds)

            if ~outOfView 
                %Append observation and append id to cam states
                featureTracks{trackedFeatureIds == featureId}.observations(:, end+1) = meas_k;
                
                %Add observation to current camera
                msckfState.camStates{end}.trackedFeatureIds(end+1) = featureId;
            end
            
            track = featureTracks{trackedFeatureIds == featureId};
            
            % 
            if outOfView ...
                    || size(track.observations, 2) >= msckfParams.maxTrackLength ...
                    || state_k+1 == kEnd
                                
                %Feature is not in view, remove from the tracked features
                [msckfState, camStates, camStateIndices] = removeTrackedFeature(msckfState, featureId);
                
                %Add the track, with all of its camStates, to the 
                %residualized list
                if length(camStates) >= msckfParams.minTrackLength
                    track.camStates = camStates;
                    track.camStateIndices = camStateIndices;
                    featureTracksToResidualize{end+1} = track;
                end
               
                %Remove the track
                featureTracks = featureTracks(trackedFeatureIds ~= featureId);
                trackedFeatureIds(trackedFeatureIds == featureId) = []; 
            end
            
        elseif ~outOfView && state_k+1 < kEnd % && ~ismember(featureId, trackedFeatureIds)
            %Track new feature
            track.featureId = featureId;
            track.observations = meas_k;
            featureTracks{end+1} = track;
            trackedFeatureIds(end+1) = featureId;

            %Add observation to current camera
            msckfState.camStates{end}.trackedFeatureIds(end+1) = featureId;
        end
    end
     
    
    
    %% ==========================FEATURE RESIDUAL CORRECTIONS======================== %%
    if ~isempty(featureTracksToResidualize)
        H_o = [];
        r_o = [];
        R_o = [];

        for f_i = 1:length(featureTracksToResidualize)

            track = featureTracksToResidualize{f_i};
            
            %Estimate feature 3D location through Gauss Newton inverse depth
            %optimization
            [p_f_G, Jcost, RCOND] = calcGNPosEst(track.camStates, track.observations, noiseParams);
            % Uncomment to use ground truth map instead
%             p_f_G = groundTruthMap(:, track.featureId); Jcost = 0; RCOND = 1;
%             p_f_C = triangulate(squeeze(y_k_j(:, track.camStates{1}.state_k, track.featureId)), camera); Jcost = 0; RCOND = 1;
        
            nObs = size(track.observations,2);
            JcostNorm = Jcost / nObs^2;
            fprintf('Jcost = %f | JcostNorm = %f | RCOND = %f\n',...
                Jcost, JcostNorm,RCOND);
            
            if JcostNorm > msckfParams.maxGNCostNorm ...
                    || RCOND < msckfParams.minRCOND
%                     || norm(p_f_G) > 50
                
                break;
            else
                map(:,end+1) = p_f_G;
                numFeatureTracksResidualized = numFeatureTracksResidualized + 1;
                fprintf('Using new feature track with %d observations. Total track count = %d.\n',...
                    nObs, numFeatureTracksResidualized);
            end
            
            %Calculate residual and Hoj 
            [r_j] = calcResidual(p_f_G, track.camStates, track.observations);
            
            R_j = diag(repmat([noiseParams.u_var_prime, noiseParams.v_var_prime], [1, numel(r_j)/2]));
            [H_o_j, A_j, H_x_j] = calcHoj(p_f_G, msckfState, track.camStateIndices);  % equ (48)

            % Stacked residuals and friends
            if msckfParams.doNullSpaceTrick
                H_o = [H_o; H_o_j];

                if ~isempty(A_j)
                    r_o_j = A_j' * r_j;
                    r_o = [r_o ; r_o_j];

                    R_o_j = A_j' * R_j * A_j;
                    R_o(end+1 : end+size(R_o_j,1), end+1 : end+size(R_o_j,2)) = R_o_j;
                end
                
            else
                H_o = [H_o; H_x_j];
                r_o = [r_o; r_j];
                R_o(end+1 : end+size(R_j,1), end+1 : end+size(R_j,2)) = R_j;
            end
        end
        
        if ~isempty(r_o)
            % Put residuals into their final update-worthy form
            
            if msckfParams.doQRdecomp
                [T_H, Q_1] = calcTH(H_o);
                r_n = Q_1' * r_o;
                R_n = Q_1' * R_o * Q_1;
            else
                T_H = H_o;
                r_n = r_o;
                R_n = R_o;
            end           
            
            % Build MSCKF covariance matrix
            P = [msckfState.imuCovar, msckfState.imuCamCovar;
                   msckfState.imuCamCovar', msckfState.camCovar];

            % Calculate Kalman gain
            K = (P*T_H') / ( T_H*P*T_H' + R_n ); % == (P*T_H') * inv( T_H*P*T_H' + R_n )

            % State correction
            deltaX = K * r_n;
            msckfState = updateState(msckfState, deltaX);

            % Covariance correction
            tempMat = (eye(15 + 6*size(msckfState.camStates,2)) - K*T_H);
%             tempMat = (eye(12 + 6*size(msckfState.camStates,2)) - K*H_o);

            P_corrected = tempMat * P * tempMat' + K * R_n * K';

            msckfState.imuCovar = P_corrected(1:15,1:15);
            msckfState.camCovar = P_corrected(16:end,16:end);
            msckfState.imuCamCovar = P_corrected(1:15, 16:end);
           
%             figure(1); clf; imagesc(deltaX); axis equal; axis ij; colorbar;
%             drawnow;
            
        end
        
    end
      %% ==========================STATE HISTORY======================== %% 
        imuStates = updateStateHistory(imuStates, msckfState, camera, state_k+1);
    
      %% ==========================STATE PRUNING======================== %%
        %Remove any camera states with no tracked features
        [msckfState, deletedCamStates] = pruneStates(msckfState);
        
        if ~isempty(deletedCamStates)
            prunedStates(end+1:end+length(deletedCamStates)) = deletedCamStates;
        end    
        
%         if max(max(msckfState.imuCovar(1:12,1:12))) > 1
%             disp('omgbroken');
%         end
        
        plot_traj;
        
    
    
end