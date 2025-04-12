clc;
clear;
close all;

% Waypoints 정의 (동일)
path = [2.00    1.00;
        4.00    1.00;
        1.25    1.75;
        5.25    8.25;
        7.25    8.75;
        11.75   10.75;
        12.00   10.00];

ref_traj = path;

% 로봇 초기 상태 설정 (동일)
robotInitialLocation = ref_traj(1,:);
robotGoal = ref_traj(end,:);
robotInitialOrientation = 0;
robotCurrentPose = [robotInitialLocation robotInitialOrientation]'; % [x; y; theta]

% 시뮬레이션 파라미터 (최대 속도 변경)
sampleTime = 0.1;
RobotMaxVel = 1.0;         % 로봇 최대 선속도 (m/s) - 문제 3 요구사항
vizRate = rateControl(1/sampleTime);

% --- 제어기 파라미터 ---
% 각속도 PI 제어 (문제 2와 동일 또는 재튜닝)
Kp_angle = 2.0;             % 각속도 비례(P) 이득
Ki_angle = 0.5;             % 각속도 적분(I) 이득
error_sum_th = 0;          % 헤딩 오차 누적 합 (초기화)

% 선속도 P 제어 (문제 3 추가)
Kpv_linear = 0.5;          % 선속도 비례(P) 이득 (튜닝 필요)

% 기타 파라미터 (동일)
waypointThreshold = 0.2;   % Waypoint 도달 판정 거리
% v_cmd_fixed = 0.3;       % 삭제됨 (이제 P 제어로 결정)
% -------------------------

frameSize = 1/0.8;

% Plot 설정
figure(1)
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); title('Waypoint Tracking with Linear(P) and Angular(PI) Control');

% 초기 Waypoint 인덱스 설정 (동일)
idx_waypoints = 2;

% 로봇 궤적 및 속도/오차 기록용 변수
robotPath = robotInitialLocation;
velocity_history = [];
error_history = [];
time_history = 0;

% 시뮬레이션 루프 (동일)
goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

while( distanceToGoal > goalRadius && idx_waypoints <= size(ref_traj, 1))

    % 현재 목표 Waypoint 좌표 (동일)
    x_target = ref_traj(idx_waypoints, 1);
    y_target = ref_traj(idx_waypoints, 2);

    % --- 제어 로직 (선속도 부분 수정) ---
    % 1. 목표 각도 계산 (theta_desired) (동일)
    theta_desired = atan2(y_target - robotCurrentPose(2), x_target - robotCurrentPose(1));

    % 2. 헤딩 오차 계산 (error_th) (동일)
    error_th = angdiff(robotCurrentPose(3), theta_desired);

    % 3. 오차 적분 업데이트 (동일)
    error_sum_th = error_sum_th + error_th * sampleTime;

    % 4. 각속도 명령 계산 (w_cmd) - PI 제어 (동일)
    w_cmd = Kp_angle * error_th + Ki_angle * error_sum_th;

    % 5. 목표 Waypoint까지 거리 계산 (선속도 제어용)
    distanceToWaypoint = norm(robotCurrentPose(1:2)' - ref_traj(idx_waypoints,:));

    % 6. 선속도 명령 계산 (v_cmd) - P 제어 및 Saturation (수정됨)
    v_cmd_raw = Kpv_linear * distanceToWaypoint; % P 제어
    v_cmd = max(0, min(RobotMaxVel, v_cmd_raw));   % 속도 제한 (0 ~ RobotMaxVel)
    % --------------------------------------

    % Waypoint 전환 로직 (거리 계산 위치 변경됨)
    % distanceToWaypoint는 위에서 이미 계산됨
    if (distanceToWaypoint < waypointThreshold)
        disp(['Waypoint ', num2str(idx_waypoints), ' reached.']);
        % error_sum_th = 0; % 필요시 적분항 리셋
        idx_waypoints = idx_waypoints + 1;
        if idx_waypoints > size(ref_traj, 1)
            disp('Last waypoint reached or passed.');
        end
    end

    % 로봇 구동 모델 (동일)
    vel = [v_cmd*cos(robotCurrentPose(3)); ...
           v_cmd*sin(robotCurrentPose(3)); ...
           w_cmd];

    % 로봇 상태 업데이트 (동일)
    robotCurrentPose = robotCurrentPose + vel * sampleTime;

    % 데이터 기록
    robotPath = [robotPath; robotCurrentPose(1:2)'];
    velocity_history = [velocity_history; v_cmd]; % 실제 적용된 선속도 기록
    error_history = [error_history; error_th];
    time_history = [time_history; time_history(end)+sampleTime];

    % 최종 목표까지의 거리 업데이트 (동일)
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));

    % --- 시각화 (타이틀 변경) ---
    hold off;
    plot(path(:,1), path(:,2),"k--d", 'MarkerSize', 8, 'LineWidth', 1);
    hold on; grid on; axis equal;
    plot(robotPath(:,1), robotPath(:,2), 'r-', 'LineWidth', 1.5);
    if idx_waypoints <= size(ref_traj, 1)
        plot(ref_traj(idx_waypoints, 1), ref_traj(idx_waypoints, 2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
    end
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 13]); ylim([0 13]);
    xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('Lin(P)/Ang(PI) Control, Target Wpt: %d, V: %.2f m/s', idx_waypoints, v_cmd)); % 속도 표시 추가
    waitfor(vizRate);
end

disp('Goal Reached!');
total_time = time_history(end);
fprintf('Total time taken: %.2f seconds\n', total_time);

% --- 결과 확인 그래프 ---
figure(2);
subplot(2,1,1);
plot(time_history(2:end), velocity_history);
xlabel('Time (s)'); ylabel('Linear Velocity (m/s)');
title('Linear Velocity Profile'); grid on; ylim([0 RobotMaxVel*1.1]); % Y축 범위 설정

subplot(2,1,2);
plot(time_history(2:end), error_history);
xlabel('Time (s)'); ylabel('Heading Error (rad)');
title('Heading Error Profile'); grid on;
