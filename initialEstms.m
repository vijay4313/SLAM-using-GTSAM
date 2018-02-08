function [ iniPts3D, iniPose] = initialEstms( DetAll, K, tags, TagSize )
% The function retrieves the camera pose from camera frame 
% projections of landmarks through the famous RPnP approach

%%for input details, check readme


% iniPose = ExtractPose(DetAll, K);

%% Get tag10 poses
iniPose = [];
tag10_3D = [0, TagSize, TagSize, 0; 0, 0, TagSize, TagSize; 0, 0, 0, 0; 1, 1, 1, 1];
gotPose = [];
iniPts3D = [];
for i = 1:length(DetAll)
    i_tags = DetAll{i};
    idx = find(i_tags(:, 1) == 10, 1);
    if ~isempty(idx)
        gotPose(length(gotPose) + 1) = i;
        tag10 = K \ [reshape(i_tags(idx, 2:end), 2, 4); ones(1, 4)];
        [iniPose(i).R, iniPose(i).T] = RPnP(tag10_3D(1:3, :), tag10(1:2, :));
        currPose = [iniPose(i).R, iniPose(i).T];
        % use pose to get world coordinated of landmarks
        s = ((K * currPose) * tag10_3D);
        lambda = mean(s(3, :));
        [~, ia, ~] = intersect(i_tags(:, 1), tags);
        i_tags_clean = i_tags(ia, :);
        [nrows, ~] = size(i_tags_clean);
        iFramePose = K * currPose;
        for itx = 1:nrows
            iFramePts = ([reshape(i_tags_clean(itx, 2:end)', 2, 4);ones(1, 4)]);
            iframe_3D = iFramePose \ (iFramePts * lambda);
            iframe_3D_norm = iframe_3D ./ iframe_3D(end, :);
            if ~any(isnan(iframe_3D_norm))
                distv = sqrt(sum((diff(iframe_3D_norm, 1, 2) .^ 2))) ./ TagSize;
                if ~any(distv > 5) 
                    iFrame_3D_clean = reshape(iframe_3D_norm(1:3, :), 12, 1)';
                    iniPts3D = [iniPts3D ; [i_tags_clean(itx, 1), iFrame_3D_clean]];
                    [~, xi, ~] = intersect(iniPts3D(:, 1), tags);
                    iniPts3D = iniPts3D(xi, :);
                end
            end
        end
    end
end

%% Get other poses
% illpose = [];

for ind = 1:length(DetAll)
    if any(gotPose == ind)
        continue
    end
    frameDat = DetAll{ind};
    [~, ix, iy] = intersect(frameDat(:, 1), iniPts3D(:, 1));
    if ~isempty(ix)
        framePts_3D = [reshape(iniPts3D(iy(1), 2:end)', 3, 4); ones(1, 4)];
        framePts = K \ [reshape(frameDat(ix(1), 2:end)', 2, 4); ones(1, 4)];
        [iniPose(ind).R, iniPose(ind).T] = RPnP(framePts_3D(1:3, :), framePts(1:2, :));
        gotPose(length(gotPose) + 1) = ind;
    end
end

%%
%Cleaning process (Keep only first occurence of tag)
% firstFrame = DetAll{1};
% firstPose = [iniPose{1}.rotation.matrix, iniPose{1}.translation.vector];
% 
% [~, ia, ~] = intersect(firstFrame(:, 1), tags);
% i_tags_clean = firstFrame(ia, :);
% iniPts3D = zeros(size(i_tags_clean, 1), 13);

% for foo = 1:size(i_tags_clean, 1)
%     proj_pts = [reshape(i_tags_clean(foo, 2:end), 2, 4); ones(1, 4)];
%     pts_3d = pinv(K * firstPose) * (proj_pts);
%     pts_clean = pts_3d(1:3, :);
%     iniPts3D(foo, :) = [i_tags_clean(foo, 1), pts_clean(:)'];  
% end

%%

rem_tags = setdiff(tags, iniPts3D(:, 1));

for foo = 1:length(rem_tags)
    for bar = 1:length(DetAll)
        barframePts = DetAll{bar};
        idx = find(barframePts(:, 1) == rem_tags(foo), 1);
        if ~isempty(idx) && ~isempty(iniPose(bar))
            proj_pts = [reshape(barframePts(idx, 2:end), 2, 4); ones(1, 4)];
            framePose = [iniPose(bar).R, iniPose(bar).T];
            if ~isempty(framePose)
                normFramePose = K * framePose;
                frame_3D = normFramePose \ (proj_pts * lambda);
                frame_3D_norm = frame_3D ./ frame_3D(end, :);
                if ~any(isnan(frame_3D_norm))
                    distv = sqrt(sum((diff(frame_3D_norm, 1, 2) .^ 2))) ./ TagSize;
                    if ~any(distv > 5) 
                        Frame_3D_clean = reshape(frame_3D_norm(1:3, :), 12, 1)';
                        iniPts3D(size(iniPts3D, 1)+1, :) = [rem_tags(foo), Frame_3D_clean];
                        break
                    end
                end
            end
        end
    end
end

%%

illpose = setdiff(1:length(DetAll), gotPose);
% 
if ~isempty(illpose)
    for itx = 1:length(illpose)
        iDat = DetAll{illpose(itx)};
        [~, ax, ay] = intersect(iDat(:, 1), iniPts3D(:, 1));
        if ~isempty(ax)
            itxPts_3D = [reshape(iniPts3D(ay(1), 2:end)', 3, 4); ones(1, 4)];
            itxPts = K \ [reshape(iDat(ax(1), 2:end)', 2, 4); ones(1, 4)];
            [iniPose(illpose(itx)).R, iniPose(illpose(itx)).T] = RPnP(itxPts_3D(1:3, :), itxPts(1:2, :));
        else
            iniPose(illpose(itx)).R = rand(3, 3);
            iniPose(illpose(itx)).T = randn(3, 1);
        end
    end
end
%            
end

