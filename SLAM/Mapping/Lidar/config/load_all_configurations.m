function config = load_all_configurations()
%LOAD_ALL_CONFIGURATIONS 모든 설정 파일을 로드하고 통합
%
% Returns:
%   config - 통합된 설정 구조체
%
% 기능:
%   - 모든 설정 파일을 중앙에서 로드
%   - 설정 검증 및 기본값 설정
%   - 성능 최적화 적용

% ========== 설정 파일 로드 ==========
try
    config.environment = environment_config();
    config.buildings = building_config();
    config.trajectory = trajectory_config();
    config.performance = performance_config();
    config.sensor = sensor_config(); % LiDAR 센서 설정 추가
    config.simulation = simulation_config();
    
    % Debug 설정을 performance에서 상위 레벨로 이동 (접근성 향상)
    config.debug = config.performance.debug;
    
    % ========== 로드 완료 메시지 ==========
    fprintf('✅ All configuration files loaded successfully.\n');
    
catch ME
    fprintf('❌ Failed to load configuration: %s\n', ME.message);
    error('Configuration loading failed');
end

% ========== 설정 검증 ==========
validate_configurations(config);

% ========== 런타임 최적화 설정 적용 ==========
if config.performance.profiling.enabled
    fprintf('📊 Profiling is enabled.\n');
end

if config.performance.memory.preallocation
    fprintf('🚀 Memory preallocation optimization is enabled.\n');
end

end

function validate_configurations(config)
%VALIDATE_CONFIGURATIONS 설정 값 검증
%
% Parameters:
%   config - 검증할 설정 구조체

% 시뮬레이션 시간 검증
assert(config.environment.simulation.time > 0, ...
    'Simulation time must be greater than 0.');

% 고도 설정 검증
assert(all(config.trajectory.spiral.altitudes > 0), ...
    'All flight altitudes must be greater than 0.');

% 빌딩 수 검증
total_buildings = length(config.buildings.zone1.buildings) + ...
                 length(config.buildings.zone2.buildings) + ...
                 length(config.buildings.zone3.buildings) + ...
                 length(config.buildings.zone4.buildings);
                 
assert(total_buildings == 12, ...
    'A total of 12 buildings must be configured. Current: %d', total_buildings);

% Sensor 설정 검증
    if ~isfield(config, 'sensor') || ~isfield(config.sensor, 'lidar')
        error('Sensor configuration is missing or incomplete');
    end
    requiredFields = {'maxRange', 'resolution', 'enabled'};
    for i = 1:length(requiredFields)
        if ~isfield(config.sensor.lidar, requiredFields{i})
            error('Required LiDAR field missing: %s', requiredFields{i});
        end
    end
    fprintf('✅ Sensor configuration validated successfully.\n');

% Simulation 설정 검증
    if ~isfield(config, 'simulation')
        error('Simulation configuration is missing');
    end
    requiredSimFields = {'map3D', 'realtime', 'logging'};
    for i = 1:length(requiredSimFields)
        if ~isfield(config.simulation, requiredSimFields{i})
            error('Required simulation field missing: %s', requiredSimFields{i});
        end
    end
    fprintf('✅ Simulation configuration validated successfully.\n');

fprintf('✅ All configuration validations completed successfully.\n');

end
