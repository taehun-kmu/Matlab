clc;
clear;
close all;

% 원본 Waypoints 정의
path = [2.00    1.00;
        4.00    1.00;
        1.25    1.75;
        2.50    2.50;
        3.25    4.00;
        4.75    4.75;
        3.75    6.50;
        5.25    8.25;
        7.25    8.75;
        9.25    10.00;
        11.75   10.75;
        12.00   10.00];

% --- 문제 4: 촘촘한 Trajectory 생성 ---
samplingDistance = 0.2; % 원하는 샘플링 간격 (m)
ref_traj_dense = path(1,:); % 시작점 추가

for i = 1:(size(path, 1) - 1)
    startPoint = path(i,:);
    endPoint = path(i+1,:);

    segmentVector = endPoint - startPoint;
    segmentLength = norm(segmentVector);

    % 세그먼트 길이가 samplingDistance보다 클 경우에만 샘플링
    if segmentLength > samplingDistance
        numPoints = round(segmentLength / samplingDistance) + 1;
        % linspace는 numPoints 개의 점을 생성 (시작점, 끝점 포함)
        x_dense = linspace(startPoint(1), endPoint(1), numPoints);
        y_dense = linspace(startPoint(2), endPoint(2), numPoints);

        % 생성된 점들 중 시작점(첫번째 점)은 이전 세그먼트의 끝점과
        % 중복되므로 제외하고 추가
        ref_traj_dense = [ref_traj_dense; [x_dense(2:end)', y_dense(2:end)']];
    else
        % 세그먼트가 샘플링 간격보다 짧거나 같으면 끝점만 추가
        ref_traj_dense = [ref_traj_dense; endPoint];
    end
end
ref_traj = ref_traj_dense; % 생성된 촘촘한 경로를 참조 경로로 사용
% -----------------------------------------

% 로봇 초기 상태 설정 (시작점은 동일, 목표점은 새로운 ref_traj의 끝점)
robotInitialLocation = ref_traj(1,:);
robotGoal = ref_traj(end,:); % 최종 목표 = 촘촘한 경로의 마지막 점
robotInitialOrientation = 0;
robotCurrentPose = [robotInitialLocation robotInitialOrientation]';

% 시뮬레이션 파라미터 (동일)
sampleTime = 0.1;
RobotMaxVel = 1.0;
RobotMinVel = 0.0;
vizRate = rateControl(1/sampleTime);

% --- 제어기 파라미터 (문제 3과 동일 또는 재튜닝 가능) ---
Kp_angle = 2.0;
Ki_angle = 0.5;
error_sum_th = 0;
Kpv_linear = 1.8; % 선속도 P 이득 (촘촘한 경로에서는 더 작게 해도 될 수 있음)

% Waypoint 도달 판정 거리 (촘촘해졌으므로 더 작게 설정 가능, 예: 0.1)
waypointThreshold = 0.1;
% --------------------------------------------------

frameSize = 1/0.8;

% Plot 설정
figure(1)
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); title('Smoother Tracking with Dense Trajectory');

% 초기 인덱스 설정 (이제 촘촘한 경로의 2번째 점을 첫 목표로)
idx_waypoints = 2;

% 로봇 궤적 및 데이터 기록용 변수 (동일)
robotPath = robotInitialLocation;
velocity_history = [];
error_history = [];
time_history = 0;

% 시뮬레이션 루프 (종료 조건은 ref_traj 기준)
goalRadius = 0.1; % 최종 목표 도달 반경 (촘촘한 경로의 마지막 점 기준)
distanceToGoal = norm(robotInitialLocation - robotGoal);

while( distanceToGoal > goalRadius && idx_waypoints <= size(ref_traj, 1))

    % 현재 목표 점 좌표 (ref_traj 사용)
    x_target = ref_traj(idx_waypoints, 1);
    y_target = ref_traj(idx_waypoints, 2);

    % --- 제어 로직 (문제 3과 동일) ---
    theta_desired = atan2(y_target - robotCurrentPose(2), x_target - robotCurrentPose(1));
    error_th = angdiff(robotCurrentPose(3), theta_desired);
    error_sum_th = error_sum_th + error_th * sampleTime;
    w_cmd = Kp_angle * error_th + Ki_angle * error_sum_th;

    distanceToWaypoint = norm(robotCurrentPose(1:2)' - ref_traj(idx_waypoints,:));
    v_cmd_raw = Kpv_linear * distanceToWaypoint;
    v_cmd = max(RobotMinVel, min(RobotMaxVel, v_cmd_raw));
    % ----------------------------------

    % Waypoint(이제는 촘촘한 점) 전환 로직 (동일)
    if (distanceToWaypoint < waypointThreshold)
        % disp(['Point ', num2str(idx_waypoints), ' reached.']); % 너무 자주 출력되므로 주석 처리
        % error_sum_th = 0; % 리셋 여부 결정
        idx_waypoints = idx_waypoints + 1;
        if idx_waypoints > size(ref_traj, 1)
            disp('End of the dense trajectory reached.');
        end
    end

    % 로봇 구동 모델 (동일)
    vel = [v_cmd*cos(robotCurrentPose(3)); ...
           v_cmd*sin(robotCurrentPose(3)); ...
           w_cmd];

    % 로봇 상태 업데이트 (동일)
    robotCurrentPose = robotCurrentPose + vel * sampleTime;

    % 데이터 기록 (동일)
    robotPath = [robotPath; robotCurrentPose(1:2)'];
    velocity_history = [velocity_history; v_cmd];
    error_history = [error_history; error_th];
    time_history = [time_history; time_history(end)+sampleTime];

    % 최종 목표까지의 거리 업데이트 (동일)
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:)); % robotGoal은 ref_traj의 마지막 점

    % --- 시각화 (참조 경로 플롯 수정) ---
    hold off;
    % 원본 Waypoint 표시 (검은색 점선/마커)
    plot(path(:,1), path(:,2),'k--d', 'MarkerSize', 8, 'LineWidth', 1);
    hold on; grid on; axis equal;
    % 촘촘한 참조 경로 표시 (파란색 실선)
    plot(ref_traj(:,1), ref_traj(:,2), 'b-', 'LineWidth', 1.5);
    % 로봇의 실제 이동 경로 표시 (빨간색 실선)
    plot(robotPath(:,1), robotPath(:,2), 'r-', 'LineWidth', 1.5);
    % 현재 목표 점 표시 (마젠타색 작은 원) - 너무 많아서 생략하거나 작게 표시
    if idx_waypoints <= size(ref_traj, 1)
         plot(ref_traj(idx_waypoints, 1), ref_traj(idx_waypoints, 2), 'mo', 'MarkerSize', 4);
    end
    % 로봇 시각화 (동일)
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 13]); ylim([0 13]);
    xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('Dense Traj Tracking (LinP/AngPI), Target Pt: %d/%d, V: %.2f m/s', idx_waypoints, size(ref_traj,1), v_cmd));
    waitfor(vizRate);
end

disp('Goal Reached!');
total_time = time_history(end);
fprintf('Total time taken: %.2f seconds\n', total_time);

% --- 결과 확인 그래프 (동일) ---
figure(2);
subplot(2,1,1);
plot(time_history(2:end), velocity_history);
xlabel('Time (s)'); ylabel('Linear Velocity (m/s)');
title('Linear Velocity Profile (Dense Trajectory)'); grid on; ylim([0 RobotMaxVel*1.1]);

subplot(2,1,2);
plot(time_history(2:end), error_history);
xlabel('Time (s)'); ylabel('Heading Error (rad)');
title('Heading Error Profile (Dense Trajectory)'); grid on;
