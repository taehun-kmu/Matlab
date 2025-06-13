function config = environment_config()
%ENVIRONMENT_CONFIG 시뮬레이션 환경 설정
%
% Returns:
%   config - 환경 설정 구조체
%
% 설정 내용:
%   - 시뮬레이션 파라미터
%   - 병렬 처리 설정
%   - 환경 기본 설정

% ========== 시뮬레이션 파라미터 ==========
config.simulation.time = 90;           % 시뮬레이션 시간 (초)
config.simulation.update_rate = 10;    % 업데이트 주기 (Hz)

% ========== 병렬 처리 설정 ==========
config.parallel.enabled = true;       % 병렬 처리 활성화
config.parallel.workers = 4;          % 워커 수

% ========== 환경 영역 설정 ==========
config.environment.area_size = 200;   % 환경 영역 크기 (m)
config.environment.floor_height = -1; % 바닥 높이 (m)
config.environment.floor_color = [0.3 0.3 0.3]; % 바닥 색상

% ========== 좌표계 설정 ==========
config.coordinate.min_boundary = 10;  % 최소 경계 (m)
config.coordinate.max_boundary = 190; % 최대 경계 (m)

% ========== 성능 최적화 설정 ==========
config.optimization.vectorization = true;    % 벡터화 사용
config.optimization.preallocation = true;   % 사전 할당 사용
config.optimization.profile = true;         % 프로파일링 활성화

end
