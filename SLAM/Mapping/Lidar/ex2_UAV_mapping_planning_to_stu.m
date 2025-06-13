%% UAV Mapping and Planning - Modular Controller (2025-06-12)
% Phase 3: SRP/모듈화/최적화 컨트롤러 (80줄 이하)

clear; clc;
close all; close all hidden;
addpath('config'); addpath('core');

% 1. Config 로드
config = load_all_configurations();

% 2. 성능 프로파일링 시작
if isfield(config, 'performance') && isfield(config.performance, 'profiling') && config.performance.profiling.enabled
    profile on
end

% 3. 병렬 처리 초기화
if isfield(config, 'environment') && isfield(config.environment, 'parallel') && config.environment.parallel.enabled && isempty(gcp('nocreate'))
    parpool('local', config.environment.parallel.workers);
end

% 4. 환경 구축
[scene, plotFrames, ax1, ax2] = environment_builder(config);

% 5. 궤적 생성
[trajectory, x_total, y_total, z_total] = trajectory_generator(config);

% 6. 시뮬레이션 실행
simResult = simulation_engine(scene, trajectory, config);

% 7. 시각화
visualization_engine(plotFrames, x_total, y_total, z_total);

% 8. 성능 프로파일링 결과
if isfield(config, 'performance') && isfield(config.performance, 'profiling') && config.performance.profiling.enabled
    profile viewer
end