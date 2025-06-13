function [lidar, map3D, totalFrames, transformCache] = initializeSimulation(scene, trajectory, config)
% 시뮬레이션 초기화 (SRP: 초기화만)

lidar = config.sensor.lidar;
map3D = config.simulation.map3D;

% totalFrames 계산
if isa(trajectory, 'waypointTrajectory') && ~isempty(trajectory.Waypoints)
    totalFrames = size(trajectory.Waypoints, 1);
else
    error('Invalid trajectory object or empty waypoints');
end

% Transform 캐시 초기화
transformCache = containers.Map('KeyType','char','ValueType','any');

% 맵 초기화
map3D = resetMap(map3D);
end

function [frameIdx, cacheHits] = processBatch(scene, lidar, frameIdx, totalFrames, config, map3D, transformCache, cacheHits)
% 한 배치 처리 (SRP: 배치 처리만)

MINI_BATCH_SIZE = config.simulation.miniBatchSize;
batchEnd = min(frameIdx + MINI_BATCH_SIZE - 1, totalFrames);
batchSize = batchEnd - frameIdx + 1;

% 미니배치 데이터 수집
miniBatchData = collectMiniBatchFrames(scene, lidar, batchSize, config);

% Transform 처리
[transforms, cacheHits] = processTransforms(scene, miniBatchData.sampleTimes, transformCache, cacheHits, config);

% 포인트 클라우드 처리
[points, validMask, sensorPoses7D] = convertTransformData(miniBatchData, transforms, config);

% 맵 업데이트
updateMapVectorized(map3D, points(validMask,:), sensorPoses7D(validMask,:), config.sensor.lidar.maxRange);

frameIdx = batchEnd + 1;
end

function results = createResults(map3D, cacheHits, totalFrames, transformCache)
% 결과 구조체 생성 (SRP: 결과 생성만)

results.map3D = map3D;
results.cacheHits = cacheHits;
results.totalFrames = totalFrames;
results.transformCache = transformCache;
end

function time = getCurrentTime(scene)
% 현재 시뮬레이션 시간 반환
if isprop(scene, 'SimulationTime')
    time = scene.SimulationTime;
else
    time = 0;
end
end

function pc = generateSimulatedPointCloud(lidar)
% 시뮬레이션용 포인트 클라우드 생성
numPoints = lidar.pointCloudSize;
points = rand(numPoints, 3) * lidar.maxRange;
pc = points;
end