function updateMapVectorized(map3D, allPoints, allSensorPoses7D, maxRange)
% 벡터화된 맵 업데이트 (SRP: 맵 업데이트만)

if isempty(allPoints) || isempty(allSensorPoses7D)
    return;
end

[uniquePoses, ~, groupIndices] = unique(allSensorPoses7D, 'rows', 'stable');

for i = 1:size(uniquePoses, 1)
    framePoints = allPoints(groupIndices == i, :);
    if isempty(framePoints)
        continue;
    end
    
    sensorPose = uniquePoses(i, :);
    if ~all(isfinite(sensorPose))
        continue;
    end
    
    ptCloud = pointCloud(framePoints);
    ptCloud = removeInvalidPoints(ptCloud);
    if ptCloud.Count == 0
        continue;
    end
    
    insertPointCloud(map3D, sensorPose, ptCloud, maxRange);
end
end

function map3D = resetMap(map3D)
% 맵 객체 초기화 (SRP: 맵 초기화만)
if ismethod(map3D, 'reset')
    map3D = map3D.reset();
end
end