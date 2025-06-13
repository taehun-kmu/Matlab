function config = trajectory_config()
%TRAJECTORY_CONFIG 궤적 생성 파라미터 설정
%
% Returns:
%   config - 궤적 설정 구조체
%
% 설정 내용:
%   - 나선형 궤적 파라미터
%   - 집중 스캔 설정
%   - 비행 제약사항

% ========== 나선형 궤적 설정 ==========
config.spiral.center_x = 100;           % 중심점 X 좌표 (m)
config.spiral.center_y = 100;           % 중심점 Y 좌표 (m)
config.spiral.outer_radius = 140;       % 외곽 반지름 (m)
config.spiral.inner_radius = 20;        % 내부 반지름 (m)
config.spiral.altitudes = [60, 80, 100]; % 비행 고도 (m)
config.spiral.num_turns = 3;            % 나선 회전 수
config.spiral.points_per_turn = 8;      % 회전당 포인트 수

% ========== 집중 스캔 설정 ==========
config.focus_scan.enabled = true;       % 집중 스캔 활성화
config.focus_scan.radius = 25;          % 집중 스캔 반지름 (m)
config.focus_scan.points_per_building = 5; % 건물당 스캔 포인트 수
config.focus_scan.buildings = [         % 집중 스캔 대상 건물 (x, y, z)
    45, 45, 70;    % Building 7 중심
    100, 45, 70;   % Building 8 중심  
    145, 70, 70    % Building 11 중심
];

% ========== 비행 제약사항 ==========
config.constraints.min_altitude = 20;   % 최소 비행 고도 (m)
config.constraints.max_altitude = 120;  % 최대 비행 고도 (m)
config.constraints.safety_margin = 10;  % 안전 여유 거리 (m)

% ========== 성능 최적화 설정 ==========
config.optimization.vectorized = true;  % 벡터화된 계산 사용
config.optimization.prealloc_factor = 1.2; % 사전 할당 여유 비율

end
