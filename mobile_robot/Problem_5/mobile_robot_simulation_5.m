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

    if segmentLength > samplingDistance
        numPoints = round(segmentLength / samplingDistance) + 1;
        x_dense = linspace(startPoint(1), endPoint(1), numPoints);
        y_dense = linspace(startPoint(2), endPoint(2), numPoints);
        ref_traj_dense = [ref_traj_dense; [x_dense(2:end)', y_dense(2:end)']];
    else
        ref_traj_dense = [ref_traj_dense; endPoint];
    end
end
ref_traj = ref_traj_dense;
% -----------------------------------------

% 로봇 초기 상태 설정
robotInitialLocation = ref_traj(1,:);
robotGoal = ref_traj(end,:);
robotInitialOrientation = 0;
robotCurrentPose = [robotInitialLocation robotInitialOrientation]';

% 시뮬레이션 파라미터
sampleTime = 0.1;
RobotMaxVel = 1.0;
RobotMinVel = 0.0;
RobotMaxAngularVel = pi; % 로봇 최대 각속도 (rad/s) - Anti-windup 및 각속도 제한용
vizRate = rateControl(1/sampleTime);

% --- 제어기 파라미터 ---
Kp_angle = 2.0;
Ki_angle = 0.5;
error_sum_th = 0;
Kpv_linear = 1.8;

% --- Look-ahead 관련 파라미터 ---
lookAheadDistance = 0.2; % (m) 로봇이 현재 위치에서 경로를 따라 내다볼 거리 (튜닝 필요)

frameSize = 1/0.8;

% Plot 설정
figure(1)
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); title('Smoother Tracking with Dense Trajectory & Look-ahead');

% 로봇 궤적 및 데이터 기록용 변수
robotPath = robotInitialLocation;
velocity_history = [];
heading_error_history = [];
cte_history = [];
time_history = 0;

% 시뮬레이션 루프
goalRadius = 0.15;
distanceToGoal = norm(robotInitialLocation - robotGoal);
approx_lookAheadIdx_for_title = 1; % 타이틀 표시용 변수

while( distanceToGoal > goalRadius )

    % --- Look-ahead Point 및 Cross-Track Error 계산 ---
    % 1. 로봇에서 가장 가까운 경로상의 점(closest_point_on_path) 및 해당 점이 속한 경로 세그먼트 찾기
    min_dist_to_path = inf;
    closest_point_on_path = robotCurrentPose(1:2)';
    currentPathSegmentStartIdx = 1; % 가장 가까운 점이 시작되는 세그먼트의 시작 인덱스

    for k = 1:(size(ref_traj,1)-1)
        pt1 = ref_traj(k,:);
        pt2 = ref_traj(k+1,:);
        vec_path_segment = pt2 - pt1;
        len_sq_path_segment = dot(vec_path_segment, vec_path_segment);

        if len_sq_path_segment == 0 % 세그먼트 길이가 0인 경우 (같은 점이 연속될 때)
            temp_closest_point = pt1;
        else
            vec_robot_to_pt1 = robotCurrentPose(1:2)' - pt1;
            projection = dot(vec_robot_to_pt1, vec_path_segment) / len_sq_path_segment;
            if projection < 0
                temp_closest_point = pt1;
            elseif projection > 1
                temp_closest_point = pt2;
            else
                temp_closest_point = pt1 + projection * vec_path_segment;
            end
        end
        dist_to_segment_point = norm(robotCurrentPose(1:2)' - temp_closest_point);
        if dist_to_segment_point < min_dist_to_path
            min_dist_to_path = dist_to_segment_point;
            closest_point_on_path = temp_closest_point;
            currentPathSegmentStartIdx = k;
        end
    end

    % CTE 계산
    path_segment_vector_for_cte = ref_traj(min(currentPathSegmentStartIdx+1, size(ref_traj,1)),:) - ref_traj(currentPathSegmentStartIdx,:);
    path_angle_for_cte = atan2(path_segment_vector_for_cte(2), path_segment_vector_for_cte(1));
    robot_to_closest_point_vec = robotCurrentPose(1:2)' - closest_point_on_path;
    normal_vector_for_cte = [-sin(path_angle_for_cte), cos(path_angle_for_cte)];
    cte = dot(robot_to_closest_point_vec, normal_vector_for_cte);

    % 2. Look-ahead Point 탐색 (수정된 로직)
    lookAheadPointFound = false;
    targetLookAheadPoint = ref_traj(end,:); % 기본값: 경로의 끝점
    
    current_search_start_idx = currentPathSegmentStartIdx; 
    dist_to_cover_on_path = lookAheadDistance;

    % 먼저, closest_point_on_path가 속한 세그먼트의 남은 부분에서 탐색
    if current_search_start_idx < size(ref_traj, 1) % 현재 세그먼트가 마지막 점이 아닌지 확인
        segment_end_of_closest_point = ref_traj(current_search_start_idx + 1, :);
        vec_closest_to_segment_end = segment_end_of_closest_point - closest_point_on_path;
        len_vec_closest_to_segment_end = norm(vec_closest_to_segment_end);

        if len_vec_closest_to_segment_end >= dist_to_cover_on_path
            if len_vec_closest_to_segment_end == 0 % 길이가 0이면 closest_point_on_path 자체가 타겟
                targetLookAheadPoint = closest_point_on_path;
            else
                targetLookAheadPoint = closest_point_on_path + (vec_closest_to_segment_end / len_vec_closest_to_segment_end) * dist_to_cover_on_path;
            end
            lookAheadPointFound = true;
            approx_lookAheadIdx_for_title = current_search_start_idx + 1;
        else
            dist_to_cover_on_path = dist_to_cover_on_path - len_vec_closest_to_segment_end;
            current_search_start_idx = current_search_start_idx + 1; % 다음 세그먼트부터 탐색 시작
        end
    else % closest_point_on_path가 경로의 마지막 점이거나 그 근처일 때
        targetLookAheadPoint = ref_traj(end,:); % 경로 끝점을 타겟으로
        lookAheadPointFound = true; % 이미 경로 끝에 도달한 것으로 간주
        approx_lookAheadIdx_for_title = size(ref_traj,1);
    end

    % 남은 거리가 있다면 다음 세그먼트들에서 계속 탐색
    if ~lookAheadPointFound && current_search_start_idx < size(ref_traj, 1)
        while current_search_start_idx < size(ref_traj, 1)
            segment_start_pt = ref_traj(current_search_start_idx, :);
            segment_end_pt = ref_traj(current_search_start_idx + 1, :);
            current_segment_vec = segment_end_pt - segment_start_pt;
            current_segment_len = norm(current_segment_vec);

            if current_segment_len >= dist_to_cover_on_path
                if current_segment_len == 0
                    targetLookAheadPoint = segment_start_pt;
                else
                    targetLookAheadPoint = segment_start_pt + (current_segment_vec / current_segment_len) * dist_to_cover_on_path;
                end
                lookAheadPointFound = true;
                approx_lookAheadIdx_for_title = current_search_start_idx + 1;
                break;
            else
                dist_to_cover_on_path = dist_to_cover_on_path - current_segment_len;
                current_search_start_idx = current_search_start_idx + 1;
            end
        end
    end
    
    % 만약 모든 경로를 탐색했음에도 lookAheadPointFound가 false이면 (경로 끝에 도달)
    % targetLookAheadPoint는 이미 ref_traj(end,:)로 설정되어 있음.
    if ~lookAheadPointFound
        approx_lookAheadIdx_for_title = size(ref_traj,1);
    end

    x_target = targetLookAheadPoint(1);
    y_target = targetLookAheadPoint(2);
    % ----------------------------------------------------

    % --- 제어 로직 ---
    theta_desired = atan2(y_target - robotCurrentPose(2), x_target - robotCurrentPose(1));
    error_th = angdiff(robotCurrentPose(3), theta_desired);
    
    potential_w_cmd_i_term = Ki_angle * (error_sum_th + error_th * sampleTime);
    potential_w_cmd_p_term = Kp_angle * error_th;
    if abs(potential_w_cmd_p_term + potential_w_cmd_i_term) > RobotMaxAngularVel && sign(error_th) == sign(error_sum_th + error_th * sampleTime)
        % I항이 포화를 악화시키고 오차와 같은 방향이면 누적하지 않음 (간단한 Anti-windup)
    else
         error_sum_th = error_sum_th + error_th * sampleTime;
    end

    w_cmd_raw = Kp_angle * error_th + Ki_angle * error_sum_th;
    w_cmd = max(-RobotMaxAngularVel, min(RobotMaxAngularVel, w_cmd_raw));

    distanceToTargetPoint = norm(robotCurrentPose(1:2)' - targetLookAheadPoint);
    v_cmd_raw = Kpv_linear * distanceToTargetPoint;
    v_cmd = max(RobotMinVel, min(RobotMaxVel, v_cmd_raw));
    % ----------------------------------

    % 로봇 구동 모델
    vel = [v_cmd*cos(robotCurrentPose(3)); ...
           v_cmd*sin(robotCurrentPose(3)); ...
           w_cmd];

    % 로봇 상태 업데이트
    robotCurrentPose = robotCurrentPose + vel * sampleTime;

    % 데이터 기록
    robotPath = [robotPath; robotCurrentPose(1:2)'];
    velocity_history = [velocity_history; v_cmd];
    heading_error_history = [heading_error_history; error_th];
    cte_history = [cte_history; cte];
    time_history = [time_history; time_history(end)+sampleTime];

    % 최종 목표까지의 거리 업데이트
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));

    % --- 시각화 ---
    hold off;
    plot(path(:,1), path(:,2),'k--d', 'MarkerSize', 8, 'LineWidth', 1);
    hold on; grid on; axis equal;
    plot(ref_traj(:,1), ref_traj(:,2), 'b-', 'LineWidth', 1.5);
    plot(robotPath(:,1), robotPath(:,2), 'r-', 'LineWidth', 1.5);
    plot(x_target, y_target, 'm*', 'MarkerSize', 8); % Look-ahead Point
    plot(closest_point_on_path(1), closest_point_on_path(2), 'go', 'MarkerSize', 6); % Closest Point

    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 13]); ylim([0 13]);
    xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('Dense Traj (LAD:%.1fm), Target Idx (approx): %d/%d, V: %.2f, CTE: %.2f', ...
        lookAheadDistance, approx_lookAheadIdx_for_title, size(ref_traj,1), v_cmd, cte));
    waitfor(vizRate);

    if norm(robotCurrentPose(1:2)' - ref_traj(end,:)) < goalRadius * 0.5 && v_cmd < 0.01 && distanceToGoal < goalRadius * 1.1
         disp('Very close to final goal with minimal speed. Ending simulation.');
         break;
    end
end

disp('Goal Reached or End of Simulation!');
total_time = time_history(end);
fprintf('Total time taken: %.2f seconds\n', total_time);
fprintf('Final distance to goal: %.2f m\n', distanceToGoal);
if ~isempty(cte_history)
    fprintf('Mean Absolute CTE: %.3f m\n', mean(abs(cte_history)));
    fprintf('Max Absolute CTE: %.3f m\n', max(abs(cte_history)));
end
if ~isempty(heading_error_history)
    fprintf('Mean Absolute Heading Error: %.3f rad\n', mean(abs(heading_error_history)));
end

% --- 결과 확인 그래프 ---
figure(2);
subplot(3,1,1);
if ~isempty(time_history) && length(time_history) > 1
    plot(time_history(2:end), velocity_history);
end
xlabel('Time (s)'); ylabel('Linear Velocity (m/s)');
title('Linear Velocity Profile'); grid on; ylim([RobotMinVel RobotMaxVel*1.1]);

subplot(3,1,2);
if ~isempty(time_history) && length(time_history) > 1
    plot(time_history(2:end), heading_error_history);
end
xlabel('Time (s)'); ylabel('Heading Error (rad)');
title('Heading Error Profile'); grid on;

subplot(3,1,3);
if ~isempty(time_history) && length(time_history) > 1
    plot(time_history(2:end), cte_history);
end
xlabel('Time (s)'); ylabel('Cross-Track Error (m)');
title('Cross-Track Error Profile'); grid on;