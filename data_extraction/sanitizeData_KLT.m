%% Clean up and import
clc;
close all;
clear;
addpath('utils');
addpath('utils/devkit');
format long;

%save(['../datasets2/' fileName], 'r_i_vk_i','theta_vk_i','w_vk_vk_i','v_vk_vk_i', 'a_vk_vk_i','cu','cv','fu','fv','b', 'y_k_j', 'C_c_v', 'rho_v_c_v', 't');
% dataBaseDir = 'C:/Users/pang/Downloads/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync';
% dataCalibDir = 'C:/Users/pang/Downloads/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync/';
dataBaseDir = 'C:/Users/pang/Downloads/MH_01_easy/mav0';

cam0dir = '/cam0/';
cam1dir = '/cam1/';
imudir = '/imu0/';
groundtruth = '/state_groundtruth_estimate0/';

% load data
load([dataBaseDir cam0dir 'imagetimestamp.mat']);
load([dataBaseDir cam0dir 'syn_index.mat']);
load([dataBaseDir imudir 'groud_aligned_imu.mat']);
load([dataBaseDir groundtruth 'state_groundtruth_estimate0.mat']);

imageListFileID = fopen([dataBaseDir cam0dir 'data.txt'],'r');
imageNameList = textscan(imageListFileID,'%s %s');




camStart = 100;
camEnd = 1200;



meanErrorHist = [];
errorVecHist = [];
bmHist = [];
imuDataHist = [];

seenFeatureVectors = [];
seenFeatureStructs = {};
allFeaturesStructIdx = []; % All of the features you have ever seen
observedIdx = [];
oldLeftPoints = [];
oldRightPoints = [];
oldFeatureIdx = [];

detectNewPoints = false;

for i = camStart:camEnd
    
    image0_name = imageNameList{1,2}{i,1};
    viLeftImage = imread([dataBaseDir cam0dir 'data/' image0_name]);
    image1_name = imageNameList{1,2}{i,1};
    viRightImage = imread([dataBaseDir cam1dir 'data/' image1_name]);
    fprintf('Processing %d %s %s\n', i,image0_name,image1_name);
    if i == camStart || detectNewPoints
        detectNewPoints = false; 
        
       points1 = detectSURFFeatures(viLeftImage);
        points2 = detectSURFFeatures(viRightImage);
        [f1, vpts1] = extractFeatures(viLeftImage, points1);
        [f2, vpts2] = extractFeatures(viRightImage, points2);
        index_pairs = matchFeatures(f1, f2) ;
        matchedPointsLeft = vpts1(index_pairs(1:20, 1));
        matchedPointsRight = vpts2(index_pairs(1:20, 2));
        figure; showMatchedFeatures(viLeftImage,viRightImage,matchedPointsLeft,matchedPointsRight,'montage');
        trackingPointsLeft =  matchedPointsLeft.Location;
        trackingPointsRight = matchedPointsRight.Location;
        
         pointTrackerL = vision.PointTracker('MaxBidirectionalError', 5);
         initialize(pointTrackerL, trackingPointsLeft, viLeftImage);
         pointTrackerR = vision.PointTracker('MaxBidirectionalError', 5);
         initialize(pointTrackerR, trackingPointsRight, viRightImage);

            %Old features
            fCount = length(seenFeatureStructs) + 1;
            for f_i = 1:size(oldLeftPoints,1)
                struct_i = oldFeatureIdx(f_i);
                seenFeatureStructs{struct_i}.leftPixels(:, end+1) = trackingPointsLeft(f_i, :)';
                seenFeatureStructs{struct_i}.rightPixels(:, end+1) = trackingPointsRight(f_i, :)';
                seenFeatureStructs{struct_i}.imageIndex(end+1) = i;
            end
            
            allFeaturesStructIdx = [oldFeatureIdx, allFeaturesStructIdx];
            
            %New features
            for obs_i = size(oldLeftPoints,1)+1:size(trackingPointsLeft,1)
                seenFeatureStructs{fCount}.leftPixels = trackingPointsLeft(obs_i, :)';
                seenFeatureStructs{fCount}.rightPixels = trackingPointsRight(obs_i, :)';
                seenFeatureStructs{fCount}.imageIndex = i;
                allFeaturesStructIdx(end+1) = fCount;
                observedIdx(end+1) = fCount;
                fCount = fCount + 1;
            end
            
             % Clear old features so we don't double count
             oldLeftPoints = [];
             oldRightPoints = [];
             oldFeatureIdx = [];

    else
         % tracking here
         [validLeftPoints, isFoundL] = step(pointTrackerL, viLeftImage);
         [validRightPoints, isFoundR] = step(pointTrackerR, viRightImage);

         trackedIdx = find(isFoundL & isFoundR & validLeftPoints(:,1) > 0 & validLeftPoints(:,2) > 0 ...
             & validRightPoints(:,1) > 0 & validRightPoints(:,2) > 0 ...
             & abs(validLeftPoints(:, 2) - validRightPoints(:, 2)) <= 1);

         observedIdx = observedIdx(trackedIdx);
         
         if length(trackedIdx) < 50
             detectNewPoints = true;
             oldLeftPoints = validLeftPoints(trackedIdx,:);
             oldRightPoints = validRightPoints(trackedIdx,:);
             oldFeatureIdx = observedIdx;
         end
         if isempty(trackedIdx)
            continue;
         end

         fprintf('Tracking %d features.', length(trackedIdx));

           setPoints(pointTrackerL, validLeftPoints(trackedIdx,:));
           setPoints(pointTrackerR, validRightPoints(trackedIdx,:)); 

        %For all previously seen features, update the feature struct to account
        %for new observation
            for f_i = 1:length(observedIdx)
                struct_i = observedIdx(f_i);
                obs_i = trackedIdx(f_i);
                seenFeatureStructs{struct_i}.leftPixels(:, end+1) = validLeftPoints(obs_i, :)';
                seenFeatureStructs{struct_i}.rightPixels(:, end+1) = validRightPoints(obs_i, :)';
                seenFeatureStructs{struct_i}.imageIndex(end+1) = i;
            end


           showMatchedFeatures(viLeftImage, viRightImage, validLeftPoints(trackedIdx,:), validRightPoints(trackedIdx,:));
    end
   
    
    
end

