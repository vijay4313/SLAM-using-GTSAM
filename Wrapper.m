%% Wrapper for P2Ph1 for CMSC828T Course at University of Maryland, College Park
% Code by: Nitin J. Sanket (nitinsan@terpmail.umd.edu)

clc
clear all
close all

%% Add ToolBox to Path eg. ToolboxPath = 'gtsam_toolbox';
%% Load Data

% Download data from the following link: 
% https://drive.google.com/drive/folders/0B6NXn7BBGQf5MS0tUGR0Nno0Nk0

load('DataMountain.mat', 'DetAll');
load('CalibParams.mat');

newDetAll = [];
for i = 1:length(DetAll)
    if any(any(DetAll{i}))
        newDetAll{length(newDetAll)+1} = DetAll{i};
    end
end


%% SLAM Using GTSAM
[LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(newDetAll, K, TagSize);
                                            
cla
plot3(LandMarksComputed(:, 2), LandMarksComputed(:, 3), LandMarksComputed(:, 4), 'r*','DisplayName', 'Tag-BLeft')
hold on
plot3(LandMarksComputed(:, 5), LandMarksComputed(:, 6), LandMarksComputed(:, 7), 'b*', 'DisplayName', 'Tag-BRight')
plot3(LandMarksComputed(:, 8), LandMarksComputed(:, 9), LandMarksComputed(:, 10), 'cyan*', 'DisplayName', 'Tag-TopLeft')
plot3(LandMarksComputed(:, 11), LandMarksComputed(:, 12), LandMarksComputed(:, 13), 'green*', 'DisplayName', 'Tag-TopRight')
plot3(AllPosesComputed(:, 1), AllPosesComputed(:, 2), AllPosesComputed(:, 3), 'blacko', 'DisplayName', 'Poses')
legend('show')
                                            
%% Localization usin iSAM2
AllPosesComputed = LocalizationUsingiSAM2(newDetAll, K, TagSize, LandMarksComputed);
                                             
 cla
 plot3(LandMarksComputed(:, 2), LandMarksComputed(:, 3), LandMarksComputed(:, 4), 'r*','DisplayName', 'Tag-BLeft')
 hold on
 plot3(LandMarksComputed(:, 5), LandMarksComputed(:, 6), LandMarksComputed(:, 7), 'b*', 'DisplayName', 'Tag-BRight')
 plot3(LandMarksComputed(:, 8), LandMarksComputed(:, 9), LandMarksComputed(:, 10), 'cyan*', 'DisplayName', 'Tag-TopLeft')
 plot3(LandMarksComputed(:, 11), LandMarksComputed(:, 12), LandMarksComputed(:, 13), 'green*', 'DisplayName', 'Tag-TopRight')
 plot3(AllPosesComputed(:, 1), AllPosesComputed(:, 2), AllPosesComputed(:, 3), 'blacko', 'DisplayName', 'Poses')
 legend('show')
                     
                                            
                                            
                                            