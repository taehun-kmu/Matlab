clc;
clear;
close all; % 이전 그림 창 닫기

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

% 시뮬레이션 파라미터 (동일)
sampleTime = 0.1;
RobotMaxVel = 1.0;
vizRate = rateControl(1/sampleTime);

% --- 문제 2: PI 제어기 파라미터 ---
Kp_angle = 2.0;             % 각속도 비례(P) 이득 (동일하게 시작)
Ki_angle = 0.5;             % 각속도 적분(I) 이득 (새로 추가, 튜닝 필요)
error_sum_th = 0;          % 헤딩 오차 누적 합 (초기화)
waypointThreshold = 0.2;   % Waypoint 도달 판정 거리 (동일)
v_cmd_fixed = 0.3;         % 고정 선속도 (동일)
% ---------------------------------

frameSize = 1/0.8;

% Plot 설정
figure(1)
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); title('Waypoint Tracking with Angular Velocity PI-Control');

% 초기 Waypoint 인덱스 설정 (동일)
idx_waypoints = 2;

% 로봇 궤적 저장을 위한 변수 (동일)
robotPath = robotInitialLocation;
% 오차 기록 (선택 사항, 비교용)
error_history = [];
time_history = 0;

% 시뮬레이션 루프 (동일)
goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

while( distanceToGoal > goalRadius && idx_waypoints <= size(ref_traj, 1))

    % 현재 목표 Waypoint 좌표 (동일)
    x_target = ref_traj(idx_waypoints, 1);
    y_target = ref_traj(idx_waypoints, 2);

    % --- 제어 로직 (수정됨) ---
    % 1. 목표 각도 계산 (theta_desired) (동일)
    theta_desired = atan2(y_target - robotCurrentPose(2), x_target - robotCurrentPose(1));

    % 2. 헤딩 오차 계산 (error_th) (동일)
    error_th = angdiff(robotCurrentPose(3), theta_desired);

    % 3. 오차 적분 업데이트 (추가됨)
    error_sum_th = error_sum_th + error_th * sampleTime;
    % (주의: 실제 시스템에서는 Anti-windup 로직이 필요할 수 있음)

    % 4. 각속도 명령 계산 (w_cmd) - PI 제어 (수정됨)
    w_cmd = Kp_angle * error_th + Ki_angle * error_sum_th;

    % 5. 선속도 명령 (v_cmd) - 고정값 (동일)
    v_cmd = v_cmd_fixed;
    % --------------------------

    % Waypoint 전환 로직 (동일)
    distanceToWaypoint = norm(robotCurrentPose(1:2)' - ref_traj(idx_waypoints,:));
    if (distanceToWaypoint < waypointThreshold)
        disp(['Waypoint ', num2str(idx_waypoints), ' reached.']);
        % Waypoint 전환 시 오차 누적값 초기화 (선택적)
        % error_sum_th = 0; % -> 로봇이 목표에 도달했으므로 과거 오차는 리셋? 상황에 따라 결정
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

    % 로봇 궤적 저장 (동일)
    robotPath = [robotPath; robotCurrentPose(1:2)'];

    % 오차 및 시간 기록 (선택 사항)
    error_history = [error_history; error_th];
    time_history = [time_history; time_history(end)+sampleTime];

    % 최종 목표까지의 거리 업데이트 (동일)
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));

    % --- 시각화 (동일, 타이틀만 수정) ---
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
    xlabel('X (m)'); ylabel('Y (m)'); title(['Waypoint Tracking (PI-Control), Target Wpt: ', num2str(idx_waypoints)]);
    waitfor(vizRate);
end

disp('Goal Reached!');

% --- 오차 확인 그래프 (선택 사항) ---
figure(2);
plot(time_history(2:end), error_history); % 첫 번째 시간 제외
title('Heading Error (e_{\theta}) over time (PI Controller)');
xlabel('Time (s)'); ylabel('Error (rad)'); grid on;

save('PI_Control.mat', 'robotPath', 'error_history', 'time_history');
