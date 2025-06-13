function config = simulation_config()
% SIMULATION_CONFIG 시뮬레이션 엔진 설정
%
% Returns:
%   config - 시뮬레이션 설정 구조체

%% ========== 3D 매핑 설정 ==========

config.map3D = occupancyMap3D(0.5); % 올바른 문법
config.mapResolution = 0.5;             % 맵 해상도 (m)
config.mapSize = [200, 200, 100];       % 맵 크기 [X, Y, Z] (m)
config.miniBatchSize = 16;              % 미니배치 크기 (병렬 최적화)

%% ========== 실시간 처리 설정 ==========
config.realtime = false;                % 실시간 처리 여부
config.realtimeMultiplier = 1.0;        % 실시간 배속

%% ========== SLAM 설정 ==========
config.slam.enabled = true;             % SLAM 활성화
config.slam.algorithm = 'lidar';        % SLAM 알고리즘
config.slam.keyframeInterval = 10;      % 키프레임 간격

%% ========== 포인트 클라우드 설정 ==========
config.pointCloud.downsample = true;    % 다운샘플링 활성화
config.pointCloud.voxelSize = 0.1;      % 복셀 크기 (m)
config.pointCloud.filtering = true;     % 필터링 활성화

%% ========== 로깅 및 디버깅 ==========
config.logging = true;                  % 로깅 활성화
config.debug = false;                   % 디버깅 모드
config.saveResults = true;              % 결과 저장

end
