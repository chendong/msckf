function [msckfState, featureTracks, trackedFeatureIds] = initializeMSCKF(firstImuState, firstMeasurements, camera, state_k, noiseParams)
%INITIALIZEMSCKF Initialize the MSCKF with tracked features and ground
%truth


%Compute the first state

msckfState.imuState = firstImuState;
msckfState.imuCovar = noiseParams.initialIMUCovar;
msckfState.camCovar = [];
msckfState.imuCamCovar = [];
msckfState.camStates = {};  %camera pose

msckfState = augmentState(msckfState, camera, state_k);

%Compute all of the relevant feature tracks
featureTracks = {};
trackedFeatureIds = [];

% walk through the squeeze of feature tracks, add the corresponding
% features index into featureId
 for featureId = 1:size(firstMeasurements.y,2)
        meas_k = firstMeasurements.y(:, featureId);
        if ~isnan(meas_k(1,1))
                %Track new feature
                track.featureId = featureId;
                track.observations = meas_k;
                %append the features in the sequence
                featureTracks{end+1} = track;
                trackedFeatureIds(end+1) = featureId;
                %Add observation to current camera
                msckfState.camStates{end}.trackedFeatureIds(end+1) = featureId;
        end
 end
 
 
end

