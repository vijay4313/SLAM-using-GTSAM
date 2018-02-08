function [isam,result, nextPoseIndex] = VisISAMStep(DetAll,iniPoints, iniPoses, noiseModels,options, isam,...
                                                    result,nextPoseIndex)
% VisualISAMStep executes one update step of visualSLAM::iSAM object
% Authors: Duy Nguyen Ta and Frank Dellaert

import gtsam.*

% iSAM expects us to give it a new set of factors 
% along with initial estimates for any new variables introduced.

%%
newFactors = NonlinearFactorGraph;
initialEstimates = Values;

for i=nextPoseIndex:nextPoseIndex+options.iniWindow
    try
        mat = DetAll{i};
    catch
        continue
    end
    i1 = symbol('x',i);
    pose_i1 = Pose3(Rot3(iniPoses(i).R), Point3(iniPoses(i).T));
    
    i0 = symbol('x', i-1);
    pose_i0 = Pose3(Rot3(iniPoses(i-1).R), Point3(iniPoses(i-1).T));
    newFactors.add(BetweenFactorPose3(i0, i1, pose_i0.between(pose_i1), noiseModels.odometry));
    
    if i==nextPoseIndex
        newFactors.add(PriorFactorPose3(i0,pose_i0, noiseModels.pose));
    end
    initialEstimates.insert(i1,pose_i1);
    
    for j = 1:size(mat, 1)
       dat = mat(j, :);
       newFactors.add(GenericProjectionFactorCal3_S2(Point2(dat(2), dat(3)), noiseModels.measurement, i1, symbol('l',dat(1)), options.K));
       newFactors.add(GenericProjectionFactorCal3_S2(Point2(dat(4), dat(5)), noiseModels.measurement, i1, symbol('m',dat(1)), options.K));
       newFactors.add(GenericProjectionFactorCal3_S2(Point2(dat(6), dat(7)), noiseModels.measurement, i1, symbol('n',dat(1)), options.K));
       newFactors.add(GenericProjectionFactorCal3_S2(Point2(dat(8), dat(9)), noiseModels.measurement, i1, symbol('o',dat(1)), options.K)); 

       if ~result.exists(symbol('l', dat(1))) && ~initialEstimates.exists(symbol('l', dat(1)))
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

isam.update(newFactors, initialEstimates);

% figure(1);tic;
% t=toc; plot(frame_i,t,'r.'); tic
result = isam.calculateEstimate();
% t=toc; plot(frame_i,t,'g.');
