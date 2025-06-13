function miniBatchData = collectMiniBatchFrames(scene, lidar, batchSize, config)
% 미니배치 프레임 데이터 수집 (SRP: 데이터 수집만)

miniBatchData.frameCount = batchSize;
miniBatchData.sampleTimes = zeros(1, batchSize);
miniBatchData.pointClouds = cell(1, batchSize);
miniBatchData.isValidFrame = false(1, batchSize);

% Config 기반 동적 시간 간격 계산
sensorPeriod = 1.0 / config.sensor.lidar.updateRate;
currentTime = getCurrentTime(scene);

for i = 1:batchSize
    miniBatchData.sampleTimes(i) = currentTime + (i-1)*sensorPeriod;
    miniBatchData.pointClouds{i} = generateSimulatedPointCloud(lidar);
    miniBatchData.isValidFrame(i) = true;
end
end

function [points, validMask, sensorPoses7D] = convertTransformData(miniBatchData, transforms, config)
% Transform 데이터 변환 (SRP: 데이터 변환만)

frameCount = miniBatchData.frameCount;
validMask = false(1, frameCount);
frameSizes = zeros(1, frameCount);
sensorPoses7D = zeros(frameCount, 7);

debugMode = isfield(config, 'debug') && isfield(config.debug, 'logging') && config.debug.logging.enabled;
invalidTransformCount = 0;

for i = 1:frameCount
    if miniBatchData.isValidFrame(i)
        frameSizes(i) = size(miniBatchData.pointClouds{i}, 1);
        T = transforms{i};
        
        if isempty(T) || any(~isfinite(T(:)))
            validMask(i) = false;
            invalidTransformCount = invalidTransformCount + 1;
            continue;
        end
        
        try
            pos = tform2trvec(T);
            quat = tform2quat(T);
            
            if all(isfinite(pos)) && all(isfinite(quat))
                sensorPoses7D(i,:) = [pos quat];
                validMask(i) = true;
            else
                validMask(i) = false;
                frameSizes(i) = 0;
                invalidTransformCount = invalidTransformCount + 1;
            end
        catch ME
            validMask(i) = false;
            frameSizes(i) = 0;
            invalidTransformCount = invalidTransformCount + 1;
            if debugMode
                fprintf('Debug: Transform conversion error at frame %d - %s\n', i, ME.message);
            end
        end
    end
end

if invalidTransformCount > 0 && debugMode
    fprintf('Warning: %d invalid transforms in batch\n', invalidTransformCount);
end

totalPointCount = sum(frameSizes);
points = zeros(totalPointCount, 3);
currentIdx = 1;
for i = 1:frameCount
    if frameSizes(i) > 0
        idxEnd = currentIdx + frameSizes(i) - 1;
        points(currentIdx:idxEnd, :) = miniBatchData.pointClouds{i};
        currentIdx = idxEnd + 1;
    end
end
end