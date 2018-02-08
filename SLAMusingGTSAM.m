function [LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                                IMU, LeftImgs, TLeftImgs, Mode)
% For Input and Output specifications refer to the readme
% The function extracts and optimizes the pose and landark position 
% through camera frame locations of landmarks

import gtsam.*
% Refer to Factor Graphs and GTSAM Introduction
% https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
% and the examples in the library in the GTSAM toolkit. See folder
% gtsam_toolbox/gtsam_examples

%% Initialize noises and other constants
measurementNoiseSigma = 1.0;
pointNoiseSigma = [0.1 0.1 0.1]';
poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';
poseNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);

K1 = Cal3_S2(K(1, 1), K(2, 2), 0, K(1,3), K(2, 3));

measurementNoise = noiseModel.Isotropic.Sigma(2,measurementNoiseSigma);
pointNoise = noiseModel.Diagonal.Sigmas(pointNoiseSigma);
graph = NonlinearFactorGraph;

initialEstimate = Values;
tags = [];
sprintf('Initializing graph')

%% Gather all tag IDs
for foo=1:length(DetAll)
    mat = DetAll{foo}(:, 1);
    tags = [tags;mat];
    tags = unique(tags);
end

%% Retrieve initial estimates for tags and Poses

[iniPoints, iniPoses] = initialEstms(DetAll, K, tags, TagSize);
missPts = setdiff(tags, iniPoints(:, 1));

%% Add all camera coordinate measurement values to factor graph

for i=1:length(DetAll)
    mat = DetAll{i};
    
    if i < length(DetAll)
        pose1 = Pose3(Rot3(iniPoses(i).R), Point3(iniPoses(i).T));
        pose2 = Pose3(Rot3(iniPoses(i+1).R), Point3(iniPoses(i+1).T));
        relPose = pose1.between(pose2);
        graph.add(BetweenFactorPose3(symbol('x', i), symbol('x', i+1), relPose, poseNoise));
    end
    
    for k=1:size(mat, 1)
        dat = mat(k, :);
        graph.add(GenericProjectionFactorCal3_S2(Point2(dat(2), dat(3)), measurementNoise, symbol('x',i), symbol('l',dat(1)), K1));
        graph.add(GenericProjectionFactorCal3_S2(Point2(dat(4), dat(5)), measurementNoise, symbol('x',i), symbol('m',dat(1)), K1));
        graph.add(GenericProjectionFactorCal3_S2(Point2(dat(6), dat(7)), measurementNoise, symbol('x',i), symbol('n',dat(1)), K1));
        graph.add(GenericProjectionFactorCal3_S2(Point2(dat(8), dat(9)), measurementNoise, symbol('x',i), symbol('o',dat(1)), K1));
    end
end

for itx = 1:length(missPts)
    initialEstimate.insert(symbol('l', missPts(itx)), Point3(randn(3, 1)));
    initialEstimate.insert(symbol('m', missPts(itx)), Point3(randn(3, 1)));
    initialEstimate.insert(symbol('n', missPts(itx)), Point3(randn(3, 1)));
    initialEstimate.insert(symbol('o', missPts(itx)), Point3(randn(3, 1)));
end

% Add prior constraint to pose and landmarks
graph.add(PriorFactorPose3(symbol('x',1), Pose3(Rot3(iniPoses(1).R), Point3(iniPoses(1).T)), poseNoise));

graph.add(PriorFactorPoint3(symbol('l', 10), Point3(0, 0, 0), pointNoise));
graph.add(PriorFactorPoint3(symbol('m', 10), Point3(0, TagSize, 0), pointNoise));
graph.add(PriorFactorPoint3(symbol('n', 10), Point3(TagSize, TagSize, 0), pointNoise));
graph.add(PriorFactorPoint3(symbol('o', 10), Point3(0, TagSize, 0), pointNoise));

% Initialize the poses and landmarks

for idx = 1:length(DetAll)
    initialEstimate.insert(symbol('x', idx), Pose3(Rot3(iniPoses(idx).R), Point3(iniPoses(idx).T)));
end

for iter = 1:length(iniPoints(:, 1))
    graph.add(BetweenFactorPoint3(symbol('l', iniPoints(iter, 1)), symbol('m', iniPoints(iter, 1)), Point3(TagSize, 0, 0), pointNoise));
    graph.add(BetweenFactorPoint3(symbol('m', iniPoints(iter, 1)), symbol('n', iniPoints(iter, 1)), Point3(0, TagSize, 0), pointNoise));
    graph.add(BetweenFactorPoint3(symbol('o', iniPoints(iter, 1)), symbol('n', iniPoints(iter, 1)), Point3(TagSize, 0, 0), pointNoise));
    graph.add(BetweenFactorPoint3(symbol('l', iniPoints(iter, 1)), symbol('o', iniPoints(iter, 1)), Point3(0, TagSize, 0), pointNoise));
    
    initialEstimate.insert(symbol('l',iniPoints(iter, 1)),Point3(iniPoints(iter, 2:4)'));
    initialEstimate.insert(symbol('m',iniPoints(iter, 1)),Point3(iniPoints(iter, 5:7)'));
    initialEstimate.insert(symbol('n',iniPoints(iter, 1)),Point3(iniPoints(iter, 8:10)'));
    initialEstimate.insert(symbol('o',iniPoints(iter, 1)),Point3(iniPoints(iter, 11:13)'));
end

%% Optimize the graph
sprintf('\nGraph made\n')

optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);

result = optimizer.optimize();
% result.print(sprintf('\nFinal result:\n  '));

% marginals = Marginals(graph, result);
% cla
% hold on;

% plot3DPoints(result, []);
% plot3DTrajectory(result, '*', 1, 8);

%% Retrieve landmarks from graph
for id = 1:length(tags)
    pl(id, :) = [result.at(symbol('l', tags(id))).x result.at(symbol('l', tags(id))).y result.at(symbol('l', tags(id))).z];
    pm(id, :) = [result.at(symbol('m', tags(id))).x result.at(symbol('m', tags(id))).y result.at(symbol('m', tags(id))).z];
    pn(id, :) = [result.at(symbol('n', tags(id))).x result.at(symbol('n', tags(id))).y result.at(symbol('n', tags(id))).z];
    po(id, :) = [result.at(symbol('o', tags(id))).x result.at(symbol('o', tags(id))).y result.at(symbol('o', tags(id))).z];
end

LandMarksComputed = [tags pl pm pn po];

%% Retrieve poses from graph
AllPoses = zeros(length(DetAll), 7);

for bar = 1:length(DetAll)
    pose = result.at(symbol('x', bar));
    rotm = pose.rotation.matrix;
    tr = pose.translation.vector;
    loc = -rotm' * tr;
    AllPoses(bar, :) = [loc',  rotm2quat(rotm)];
end

%% Remove inconsistent poses

AllPosesComputed(1, :) = AllPoses(1, :);

for pntr = 2:length(DetAll)
    point1 = AllPoses(pntr, 1:3);
    point0 = AllPosesComputed(size(AllPosesComputed, 1), 1:3);
    distV = norm(point0 - point1);
    if distV < 1
        AllPosesComputed(size(AllPosesComputed, 1)+1, :) = AllPoses(pntr, :);
    end
end

% 
% means = mean(AllPoses2(:, 1:3));
% sd = std(AllPoses2(:, 1:3));
% 
% t1 = abs(means+3*sd);
% t2 = abs(means-3*sd);
% 
% for qq = 1:size(AllPoses2, 1)
%     pose1 = AllPoses2(qq, 1:3);
%     if all(abs(pose1) < t1) && all(abs(pose1) > t2)
%         AllPosesComputed(qq, :) = pose1;
%     end
% end
% 
% AllPosesComputed = AllPosesComputed(any(AllPosesComputed, 2), :);
% 
% plot3(AllPosesComputed(:, 1), AllPosesComputed(:, 2), AllPosesComputed(:, 3))
% 
% plot3(points(:, 2), points(:, 3), points(:, 4), 'r*')
% hold on
% plot3(points(:, 5), points(:, 6), points(:, 7), 'b*')
% hold on
% plot3(points(:, 8), points(:, 9), points(:, 10), 'black*')
% hold on
% plot3(points(:, 11), points(:, 12), points(:, 13), 'green*')

end
