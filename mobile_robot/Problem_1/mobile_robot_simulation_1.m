clc;
clear;
close all; % 이전 그림 창 닫기

% Waypoints 정의
path = [2.00    1.00;
        4.00    1.00;
        1.25    1.75;
        5.25    8.25;
        7.25    8.75;
        11.75   10.75;
        12.00   10.00];

ref_traj = path; % 참조 경로는 waypoints 사용

% 로봇 초기 상태 설정
robotInitialLocation = ref_traj(1,:); % 시작 위치 = 첫 번째 waypoint
robotGoal = ref_traj(end,:);         % 최종 목표 = 마지막 waypoint
robotInitialOrientation = 0;         % 초기 방향 (rad)
robotCurrentPose = [robotInitialLocation robotInitialOrientation]'; % [x; y; theta]

% 시뮬레이션 파라미터
sampleTime = 0.1;          % 샘플링 시간 (s)
RobotMaxVel = 1.0;         % 로봇 최대 선속도 (m/s) - 여기서는 사용 안 함
vizRate = rateControl(1/sampleTime); % 시각화 속도 제어

% --- 문제 1: P 제어기 파라미터 ---
Kp_angle = 2.0;             % 각속도 P 제어 이득 (Kp)
waypointThreshold = 0.2;   % Waypoint 도달 판정 거리 (m)
v_cmd_fixed = 0.3;         % 고정 선속도 (m/s)
% ---------------------------------

frameSize = 1/0.8;         % 시각화 로봇 크기

% Plot 설정
figure(1)
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); title('Waypoint Tracking with Angular Velocity P-Control');

% 초기 Waypoint 인덱스 설정 (첫번째 목표는 2번 waypoint)
idx_waypoints = 2;

% 로봇 궤적 저장을 위한 변수
robotPath = robotInitialLocation;

% --- 오차 확인 (선택 사항) ---
% 시뮬레이션 동안 error_th 를 저장하여 플롯하면 오차가 어떻게 변하는지 확인할 수 있습니다.
error_history = []; % 루프 전에 초기화

% 시뮬레이션 루프
goalRadius = 0.1; % 최종 목표 도달 반경
distanceToGoal = norm(robotInitialLocation - robotGoal);

while( distanceToGoal > goalRadius && idx_waypoints <= size(ref_traj, 1))

    % 현재 목표 Waypoint 좌표
    x_target = ref_traj(idx_waypoints, 1);
    y_target = ref_traj(idx_waypoints, 2);

    % --- 제어 로직 ---
    % 1. 목표 각도 계산 (theta_desired)
    theta_desired = atan2(y_target - robotCurrentPose(2), x_target - robotCurrentPose(1));

    % 2. 헤딩 오차 계산 (error_th)
    error_th = angdiff(robotCurrentPose(3), theta_desired); % angdiff(current, desired) = desired - current

    % 3. 각속도 명령 계산 (w_cmd) - P 제어
    w_cmd = Kp_angle * error_th;

    % 4. 선속도 명령 (v_cmd) - 고정값
    v_cmd = v_cmd_fixed;
    % ----------------

    error_history = [error_history; error_th]; % 루프 안에서 저장

    % Waypoint 전환 로직
    distanceToWaypoint = norm(robotCurrentPose(1:2)' - ref_traj(idx_waypoints,:));
    if (distanceToWaypoint < waypointThreshold)
        disp(['Waypoint ', num2str(idx_waypoints), ' reached.']);
        idx_waypoints = idx_waypoints + 1;
        if idx_waypoints > size(ref_traj, 1)
            % 마지막 Waypoint에 도달하면 루프 종료 조건을 위해 idx_waypoints를 유지하거나
            % 혹은 마지막 점을 계속 목표하도록 설정 (현재는 루프 조건에서 처리)
            disp('Last waypoint reached or passed.');
        end
    end

    % 로봇 구동 모델 (선속도, 각속도 -> x_dot, y_dot, theta_dot)
    % 속도 제한은 여기서는 고정 속도를 사용하므로 필요 없지만, 일반적인 경우를 위해 남겨둠
    % v_cmd = max(0, min(RobotMaxVel, v_cmd)); % 로봇의 실제 속도를 0 - RobotMaxVel로 제한
    vel = [v_cmd*cos(robotCurrentPose(3)); ... % x_dot
           v_cmd*sin(robotCurrentPose(3)); ... % y_dot
           w_cmd];                          % theta_dot

    % 로봇 상태 업데이트 (오일러 적분)
    robotCurrentPose = robotCurrentPose + vel * sampleTime;

    % 로봇 궤적 저장
    robotPath = [robotPath; robotCurrentPose(1:2)'];

    % 최종 목표까지의 거리 업데이트
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));

    % --- 시각화 ---
    hold off; % 이전 프레임 지우기

    % 기준 경로 플롯 (점선과 마커)
    plot(path(:,1), path(:,2),"k--d", 'MarkerSize', 8, 'LineWidth', 1);
    hold on; grid on; axis equal; % 축과 그리드 다시 설정

    % 로봇의 이동 경로 플롯 (빨간색 실선)
    plot(robotPath(:,1), robotPath(:,2), 'r-', 'LineWidth', 1.5);

    % 현재 목표 Waypoint 표시 (파란색 원)
    if idx_waypoints <= size(ref_traj, 1)
        plot(ref_traj(idx_waypoints, 1), ref_traj(idx_waypoints, 2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
    end

    % 로봇 현재 위치 및 방향 시각화
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light; % 조명 추가
    xlim([0 13]);
    ylim([0 13]);
    xlabel('X (m)'); ylabel('Y (m)'); title(['Waypoint Tracking (P-Control), Target Wpt: ', num2str(idx_waypoints)]);

    waitfor(vizRate); % 시뮬레이션 속도 맞추기
end

disp('Goal Reached!');

time_vector = (0:length(error_history)-1) * sampleTime;
figure(2);
plot(time_vector, error_history);
title(['Heading Error (e_{\theta}) over time (Kp = ', num2str(Kp_angle), ')']);
xlabel('Time (s)');
ylabel('Error (rad)');
grid on;

save('P_Control.mat', 'robotPath', 'error_history');
