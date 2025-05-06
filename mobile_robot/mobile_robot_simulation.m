clc;
clear;

path = [2.00    1.00; 
        1.25    1.75;
        5.25    8.25;
        7.25    8.75;
        11.75   10.75;
        12.00   10.00];

ref_traj = path;

robotInitialLocation = ref_traj(1, :);
robotGoal = ref_traj(end, :);

robotInitialOrientation = 0;
robotCurrentPose = [robotInitialLocation robotInitialOrientation]';

sampleTime = 0.1;
RobotMaxVel = 1.0;
vizRate = rateControl(1 / sampleTime);

LookAheadDist = 0.3;

% P 제어기 게인 설정
Kp = 0.4;  % P 제어기 게인 값

frameSize = 1 / 0.8;

% 오차 로깅을 위한 변수
heading_errors = [];
time_stamps = [];
time_elapsed = 0;

% Plot 설정
figure(1)
hold all

% Control Parameter
goalRadius = 1;
distanceToGoal = norm(robotInitialLocation - robotGoal);
idx_waypoints = 1;

while(distanceToGoal > goalRadius && idx_waypoints <= size(ref_traj, 1))
    % 목표 위치
    x_desired = ref_traj(idx_waypoints, 1);
    y_desired = ref_traj(idx_waypoints, 2);
    
    % 로봇 현재 위치
    x = robotCurrentPose(1);
    y = robotCurrentPose(2);
    theta = robotCurrentPose(3);
    
    % 선속도 고정 (0.3 m/s)
    v_cmd = 0.3;
    
    % 각속도 P 제어기 - 문제 1
    theta_desired = atan2(y_desired - y, x_desired - x);
    error_th = angdiff(theta, theta_desired);
    w_cmd = Kp * error_th;  % 각속도에 대한 P 제어 (Kp = 0.4)
    
    % 오차 로깅
    heading_errors = [heading_errors; error_th];
    time_stamps = [time_stamps; time_elapsed];
    time_elapsed = time_elapsed + sampleTime;
    
    % 웨이포인트 전환 확인
    distanceToWaypoint = norm([x, y] - ref_traj(idx_waypoints, :));
    if(distanceToWaypoint < LookAheadDist)
        if(idx_waypoints < size(ref_traj, 1))
            idx_waypoints = idx_waypoints + 1;
        end
    end
    
    % 로봇 이동
    vel = [v_cmd * cos(theta); v_cmd * sin(theta); w_cmd];
    robotCurrentPose = robotCurrentPose + vel * sampleTime;
    
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % 시각화
    hold off
    plot(path(:,1), path(:,2), "k--d")
    hold all
    
    % 현재 타겟 웨이포인트 표시
    plot(x_desired, y_desired, 'ro', 'MarkerSize', 10);
    
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View", "2D", "FrameSize", frameSize);
    light;
    xlim([0 13])
    ylim([0 13])
    
    title(['Waypoint: ' num2str(idx_waypoints) ', Heading Error: ' num2str(error_th, '%.2f') ' rad']);
    
    waitfor(vizRate);
end

% 오차 분석 그래프
figure(2)
plot(time_stamps, heading_errors, 'b-');
title('Heading Error vs. Time (P Controller Only)');
xlabel('Time (s)');
ylabel('Heading Error (rad)');
grid on;

% 최종 오차 결과 출력
disp(['P Controller (Kp = ', num2str(Kp), ') 결과:']);
disp(['Maximum heading error: ', num2str(max(abs(heading_errors))), ' rad']);
disp(['Minimum heading error: ', num2str(mean(abs(heading_errors))), ' rad']);
