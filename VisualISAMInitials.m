function [noiseModels,isam,result,nextPoseIndex] = VisualISAMInitials(DetAll,iniPoints, iniPoses,options)
% VisualISAMInitialize initializes visualSLAM::iSAM object and noise parameters
% Authors: Duy Nguyen Ta, Frank Dellaert and Alex Cunningham

import gtsam.*

%% Initialize iSAM
params = gtsam.ISAM2Params;
% if options.alwaysRelinearize
%     params.setRelinearizeSkip(1);
% end
isam = ISAM2(params);

%% Set Noise parameters
noiseModels.pose = noiseModel.Diagonal.Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
%noiseModels.odometry = noiseModel.Diagonal.Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
noiseModels.odometry = noiseModel.Diagonal.Sigmas([0.05 0.05 0.05 0.1 0.1 0.1]');
noiseModels.point = noiseModel.Isotropic.Sigma(3, 0.1);
noiseModels.measurement = noiseModel.Isotropic.Sigma(2, 1.0);

%% Add constraints/priors
% TODO: should not be from ground truth!
newFactors = NonlinearFactorGraph;
initialEstimates = Values;
for i=1:options.iniWindow
    i1 = symbol('x',i);
    pose_i1 = Pose3(Rot3(iniPoses(i).R), Point3(iniPoses(i).T));
    if i==1
        newFactors.add(PriorFactorPose3(i1,pose_i1, noiseModels.pose));
    end
    initialEstimates.insert(i1,pose_i1);
    mat = DetAll{i};
    
    if i < options.iniWindow
        i2 = symbol('x', i+1);
        pose_i2 = Pose3(Rot3(iniPoses(i+1).R), Point3(iniPoses(i+1).T));
        newFactors.add(BetweenFactorPose3(i1, i2, pose_i1.between(pose_i2), noiseModels.odometry));
    end
    
    for j = 1:size(mat, 1)
       dat = mat(j, :);
       newFactors.add(GenericProjectionFactorCal3_S2(Point2(dat(2), dat(3)), noiseModels.measurement, i1, symbol('l',dat(1)), options.K));
       newFactors.add(GenericProjectionFactorCal3_S2(Point2(dat(4), dat(5)), noiseModels.measurement, i1, symbol('m',dat(1)), options.K));
       newFactors.add(GenericProjectionFactorCal3_S2(Point2(dat(6), dat(7)), noiseModels.measurement, i1, symbol('n',dat(1)), options.K));
       newFactors.add(GenericProjectionFactorCal3_S2(Point2(dat(8), dat(9)), noiseModels.measurement, i1, symbol('o',dat(1)), options.K)); 
       if ~initialEstimates.exists(symbol('l', dat(1)))
           pts = iniPoints(iniPoints(:, 1) == dat(1), 2:end);
           initialEstimates.insert(symbol('l', dat(1)), Point3(pts(1:3)'));
           initialEstimates.insert(symbol('m', dat(1)), Point3(pts(4:6)'));
           initialEstimates.insert(symbol('n', dat(1)), Point3(pts(7:9)'));
           initialEstimates.insert(symbol('o', dat(1)), Point3(pts(10:12)'));
           
           newFactors.add(PriorFactorPoint3(symbol('l', dat(1)), Point3(pts(1:3)'), noiseModels.point));
           newFactors.add(PriorFactorPoint3(symbol('m', dat(1)), Point3(pts(4:6)'), noiseModels.point));
           newFactors.add(PriorFactorPoint3(symbol('n', dat(1)), Point3(pts(7:9)'), noiseModels.point));
           newFactors.add(PriorFactorPoint3(symbol('o', dat(1)), Point3(pts(10:12)'), noiseModels.point));
       end
    end
end

nextPoseIndex = i+1;

%% Update ISAM
batchOptimizer = LevenbergMarquardtOptimizer(newFactors, initialEstimates);
fullyOptimized = batchOptimizer.optimize();
isam.update(newFactors, fullyOptimized);

% figure(1);tic;
% t=toc; plot(frame_i,t,'r.'); tic
result = isam.calculateEstimate();
% t=toc; plot(frame_i,t,'g.');

