function [trajectory, x_total, y_total, z_total] = trajectory_generator(config)
%% TRAJECTORY_GENERATOR - 고성능 궤적 생성 모듈
% Phase 3 Ultrathink: SRP 준수 궤적 생성 전용 모듈
% 책임: 다층 나선형 궤적, 집중 스캔 포인트, waypointTrajectory 생성
%
% 입력:
%   config - 궤적 설정 구조체 (trajectory, environment 포함)
%
% 출력:
%   trajectory - waypointTrajectory 객체
%   x_total, y_total, z_total - 궤적 좌표 벡터
%
% 성능 최적화:
%   ✅ 완전 벡터화된 나선형 계산
%   ✅ 사전 할당된 메모리
%   ✅ 배치 벡터 연산

%% 1단계: Config 기반 파라미터 추출
outer_radius = config.trajectory.spiral.outer_radius;
center_x = config.trajectory.spiral.center_x;
center_y = config.trajectory.spiral.center_y;
altitudes = config.trajectory.spiral.altitudes;
num_turns = config.trajectory.spiral.num_turns;
points_per_turn = config.trajectory.spiral.points_per_turn;
focus_points_estimate = size(config.trajectory.focus_scan.buildings, 1) * config.trajectory.focus_scan.points_per_building;
simTime = config.environment.simulation.time;
updateRate = config.environment.simulation.update_rate;

%% 2단계: 벡터화된 나선형 궤적 계산
points_per_spiral = num_turns * points_per_turn;
total_spiral_points = length(altitudes) * points_per_spiral;
total_points = total_spiral_points + focus_points_estimate;

% 기본 나선형 패턴 생성 (벡터화)
theta_base = linspace(0, 2*pi*num_turns, points_per_spiral);
radius_base = linspace(outer_radius, config.trajectory.spiral.inner_radius, points_per_spiral);

% 모든 고도에 대한 벡터화된 확장
theta_all = repmat(theta_base, 1, length(altitudes));
radius_all = repmat(radius_base, 1, length(altitudes));
alt_all = kron(altitudes, ones(1, points_per_spiral));

% 좌표 계산 (완전 벡터화)
x_spiral_all = center_x + radius_all .* cos(theta_all);
y_spiral_all = center_y + radius_all .* sin(theta_all);
z_spiral_all = alt_all;

% 경계 체크 (벡터화)
x_spiral_all = max(10, min(190, x_spiral_all));
y_spiral_all = max(10, min(190, y_spiral_all));

%% 3단계: 메모리 사전 할당 및 데이터 할당
x_total = zeros(1, total_points);
y_total = zeros(1, total_points);
z_total = zeros(1, total_points);

% 나선형 데이터 할당
x_total(1:total_spiral_points) = x_spiral_all;
y_total(1:total_spiral_points) = y_spiral_all;
z_total(1:total_spiral_points) = z_spiral_all;

%% 4단계: 집중 스캔 포인트 생성 (벡터화 최적화)
point_idx = total_spiral_points + 1;

% 건물 집중 스캔 대상 정의
focus_buildings = [
    45, 45, 70;   % Building 7 중심
    100, 45, 70;  % Building 8 중심
    145, 70, 70   % Building 11 중심
];

% 집중 스캔 파라미터
focus_radius = 25;
focus_points_per_building = 5;

% 벡터화된 집중 스캔 포인트 계산
angles = linspace(0, 2*pi, focus_points_per_building+1);
angles = angles(1:end-1);

num_buildings = size(focus_buildings, 1);
num_angles = length(angles);

% 메쉬그리드를 이용한 벡터화
[buildings_idx, angles_idx] = meshgrid(1:num_buildings, 1:num_angles);
buildings_idx = buildings_idx(:);
angles_idx = angles_idx(:);

% 집중 스캔 좌표 계산 (벡터화)
focus_x_vec = focus_buildings(buildings_idx, 1) + focus_radius * cos(angles(angles_idx));
focus_y_vec = focus_buildings(buildings_idx, 2) + focus_radius * sin(angles(angles_idx));
focus_z_vec = focus_buildings(buildings_idx, 3);

% 경계 체크 (벡터화)
focus_x_vec = max(10, min(190, focus_x_vec));
focus_y_vec = max(10, min(190, focus_y_vec));

%% 5단계: 집중 스캔 포인트 할당 (배열 범위 안전성)
num_focus_points = length(focus_x_vec);
end_focus_idx = min(point_idx + num_focus_points - 1, total_points);
actual_focus_points = end_focus_idx - point_idx + 1;

if actual_focus_points > 0
    x_total(point_idx:end_focus_idx) = focus_x_vec(1:actual_focus_points);
    y_total(point_idx:end_focus_idx) = focus_y_vec(1:actual_focus_points);
    z_total(point_idx:end_focus_idx) = focus_z_vec(1:actual_focus_points);
    point_idx = end_focus_idx + 1;
end

%% 6단계: 배열 크기 최적화 및 궤적 객체 생성
actual_points = point_idx - 1;
x_total = x_total(1:actual_points);
y_total = y_total(1:actual_points);
z_total = z_total(1:actual_points);

% waypointTrajectory 생성
num_waypoints = actual_points;
waypoints = [x_total', y_total', z_total'];

% 방향 벡터 (균일 할당)
orientation_vec = repmat(quaternion([0 0 0 1]), num_waypoints, 1);

% 시간 벡터 (균등 분배)
time = linspace(0, simTime, num_waypoints);

% waypointTrajectory 객체 생성
trajectory = waypointTrajectory('Waypoints', waypoints, 'Orientation', orientation_vec, ...
    'SampleRate', updateRate, 'ReferenceFrame', 'ENU', 'TimeOfArrival', time);

fprintf('✅ Trajectory Generator: Trajectory generation complete\n');
fprintf('   - Total points: %d (Spiral: %d, Focus: %d)\n', ...
    actual_points, total_spiral_points, actual_focus_points);
fprintf('   - Altitude levels: %d (%s)\n', length(altitudes), mat2str(altitudes));
fprintf('   - Performance optimization: full vectorization and pre-allocation applied\n');

end
