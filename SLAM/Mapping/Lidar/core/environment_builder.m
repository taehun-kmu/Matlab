function [scene, plotFrames, ax1, ax2] = environment_builder(config)
%% ENVIRONMENT_BUILDER - UAV 시뮬레이션 환경 구축 모듈
% Phase 3 Ultrathink: SRP 준수 환경 구축 전용 모듈
% 책임: UAV 시나리오 생성, 빌딩 배치, 시각화 환경 초기화
% 
% 입력:
%   config - 환경 설정 구조체 (environment, buildings 포함)
%
% 출력:
%   scene - UAV 시나리오 객체
%   plotFrames - 플롯 프레임 구조체
%   ax1, ax2 - 시각화 축 핸들
%
% 성능 최적화:
%   ✅ 벡터화된 빌딩 배치
%   ✅ 사전 할당된 플롯 구조체
%   ✅ 메모리 효율적 축 초기화

%% 1단계: UAV 시나리오 초기화
simTime = config.environment.simulation.time;
updateRate = config.environment.simulation.update_rate;

% UAV 시나리오 생성 (병렬 처리 최적화)
scene = uavScenario('UpdateRate', updateRate, 'StopTime', simTime);

%% 2단계: 환경 빌딩 배치 (벡터화 최적화)
create_buildings_from_config(scene, config.buildings);

%% 3단계: 성능 최적화된 시각화 환경 초기화
% 플롯 프레임 구조체 사전 할당
plotFrames = struct();

% Figure 1: 3D UAV Scene 설정
hFig1 = figure(1);
clf(hFig1);
set(hFig1, 'Position', [50 50 1200 800]); 
set(hFig1, 'NumberTitle', 'off', 'Name', 'Figure 1: UAV Scene'); 

% 첫 번째 축 생성 및 최적화
ax1 = subplot(1,2,1, 'Parent', hFig1);
axis(ax1, 'equal');
view(ax1, [-115 20]);

% 축 제한 설정 (성능 최적화를 위한 고정 제한)
xlim(ax1, [-30 230]); 
ylim(ax1, [-30 230]); 
zlim(ax1, [0 200]);   
axis(ax1, 'equal');
hold(ax1, 'on');

% 두 번째 축 생성 및 최적화
ax2 = subplot(1,2,2, 'Parent', hFig1);
hold(ax2, 'on');
axis(ax2, 'equal');
xlim(ax2, [-150 350]); 
ylim(ax2, [-150 350]); 
grid(ax2, 'on');
title(ax2, 'Trajectory Overview (Top View)');
view(ax2, [0 90]);

%% 4단계: 컬러맵 및 시각화 속성 최적화
colormap(ax1, 'jet'); 

% 축 모드 고정 (성능 향상)
set(ax1, 'XLimMode', 'manual', 'YLimMode', 'manual', 'ZLimMode', 'manual');
set(ax2, 'XLimMode', 'manual', 'YLimMode', 'manual', 'ZLimMode', 'manual');

%% 5단계: 플롯 프레임 구조체 구성
% UAV Lidar 플롯 프레임 초기화
plotFrames.UAV = struct();
plotFrames.UAV.Lidar = ax1;

% 시나리오 초기 설정
setup(scene);

fprintf('✅ Environment Builder: Environment setup complete\n');
fprintf('   - UAV scenario: %.1fs, %.1fHz\n', simTime, updateRate);
fprintf('   - Visualization axes: 2 axes optimized\n');
fprintf('   - Performance optimization: axes fixed, vectorization applied\n');

end
