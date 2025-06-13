function results = simulation_engine(scene, trajectory, config)
% 시뮬레이션 메인 조율 함수 (SRP: 조율만)

% 초기화
[lidar, map3D, totalFrames, transformCache] = initializeSimulation(scene, trajectory, config);

% 시뮬레이션 루프 (조율만)
cacheHits = 0;
frameIdx = 1;
while frameIdx <= totalFrames
    [frameIdx, cacheHits] = processBatch(scene, lidar, frameIdx, totalFrames, config, map3D, transformCache, cacheHits);
end

% 결과 반환
results = createResults(map3D, cacheHits, totalFrames, transformCache);
end


