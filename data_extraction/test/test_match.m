
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
viLeftImage = imread('left.png');
viRightImage = imread('right.png');
points1 = detectSURFFeatures(viLeftImage);
points2 = detectSURFFeatures(viRightImage);
[f1, vpts1] = extractFeatures(viLeftImage, points1);
[f2, vpts2] = extractFeatures(viRightImage, points2);
index_pairs = matchFeatures(f1, f2) ;
matched_pts1 = vpts1(index_pairs(1:20, 1));
matched_pts2 = vpts2(index_pairs(1:20, 2));
figure; showMatchedFeatures(viLeftImage,viRightImage,matched_pts1,matched_pts2,'montage');
