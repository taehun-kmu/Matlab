function config = sensor_config()
% SENSOR_CONFIG LiDAR 센서 설정
% 
% Returns:
%   config - 센서 설정 구조체
%
% 호환성:
%   - MATLAB Robotics System Toolbox
%   - UAV Scenario Simulator
%   - Point Cloud Processing
%
% 작성일: 2025-06-12
% 최적화: 메모리 사전 할당 지원

%% ========== 기본 LiDAR 파라미터 ==========
config.lidar.maxRange = 100;              % 최대 감지 거리 (m)
config.lidar.resolution = 0.5;            % 각도 해상도 (도)
config.lidar.fieldOfView = 360;           % 수평 시야각 (도)
config.lidar.verticalFieldOfView = 30;    % 수직 시야각 (도, ±15도)
config.lidar.verticalResolution = 1;      % 수직 해상도 (도)
config.lidar.minRange = 0.1;              % 최소 감지 거리 (m)

%% ========== 노이즈 및 정확도 설정 ==========
config.lidar.noiseVariance = 0.01;        % 거리 측정 노이즈 분산
config.lidar.angularNoise = 0.001;        % 각도 노이즈 (라디안)
config.lidar.intensity = true;            % 반사 강도 측정 여부

%% ========== 성능 및 시뮬레이션 설정 ==========
config.lidar.updateRate = 10;             % 업데이트 주기 (Hz)
config.lidar.pointCloudSize = 1000;       % 예상 포인트 클라우드 크기
config.lidar.enabled = true;              % 센서 활성화 여부

%% ========== 고급 설정 ==========
config.lidar.detectionMode = 'strongest'; % 'strongest', 'last', 'dual'
config.lidar.maxDetections = 3;           % 다중 반사 최대 감지 수
config.lidar.rangeAccuracy = 0.03;        % 거리 정확도 (m)

%% ========== 메모리 최적화 설정 ==========
config.lidar.preallocateMemory = true;    % 메모리 사전 할당 사용
config.lidar.maxBufferSize = 200;         % 최대 버퍼 크기

% 파라미터 유효성 검사
assert(config.lidar.maxRange > config.lidar.minRange, ...
    'maxRange must be greater than minRange');
assert(config.lidar.resolution > 0 && config.lidar.resolution <= 360, ...
    'resolution must be between 0 and 360 degrees');
assert(config.lidar.updateRate > 0 && config.lidar.updateRate <= 100, ...
    'updateRate must be between 0 and 100 Hz');

end
