clc;
clear;
close all;

%% 1. 경로 정의 (예시 경로)
% 원본 Waypoint
path_original = [ 1, 1;
                  3, 2;
                  4, 4;
                  6, 5;
                  8, 3;
                  9, 1];

% 촘촘한 경로 생성 (문제 4 방식 활용)
samplingDistance = 0.1; % 경로를 더 부드럽게 만들기 위한 샘플링 간격
path_dense = path_original(1,:);
for i = 1:(size(path_original, 1) - 1)
    startPoint = path_original(i,:);
    endPoint = path_original(i+1,:);
    segmentVector = endPoint - startPoint;
    segmentLength = norm(segmentVector);
    if segmentLength > samplingDistance
        numPoints = round(segmentLength / samplingDistance) + 1;
        x_dense = linspace(startPoint(1), endPoint(1), numPoints);
        y_dense = linspace(startPoint(2), endPoint(2), numPoints);
        path_dense = [path_dense; [x_dense(2:end)', y_dense(2:end)']];
    else
        path_dense = [path_dense; endPoint];
    end
end

%% 2. 로봇 상태 및 Look-ahead Distance 정의
robotX = 2.5;       % 로봇의 현재 X 위치
robotY = 1.5;       % 로봇의 현재 Y 위치
robotTheta = pi/4;  % 로봇의 현재 방향 (rad)
robotPose = [robotX, robotY, robotTheta];

lookAheadDist = 1.5; % Look-ahead Distance (d*)

%% 3. 가장 가까운 경로 점 찾기 (Closest Point)
distances = sqrt(sum((path_dense - [robotX, robotY]).^2, 2));
[minDist, closestIdx] = min(distances);
closestPoint = path_dense(closestIdx, :);

%% 4. Look-ahead Point 찾기
% closestPoint부터 경로를 따라 lookAheadDist 만큼 떨어진 점 찾기
currentDist = 0;
lookAheadIdx = closestIdx; % 시작 인덱스

% 경로 끝까지 탐색
while lookAheadIdx < size(path_dense, 1)
    % 다음 점까지의 거리
    distToNext = norm(path_dense(lookAheadIdx+1, :) - path_dense(lookAheadIdx, :));

    % 누적 거리가 lookAheadDist를 넘으면 해당 세그먼트 내에 존재
    if currentDist + distToNext >= lookAheadDist
        % 정확한 위치 보간 (Interpolation)
        remainingDist = lookAheadDist - currentDist;
        segmentVector = path_dense(lookAheadIdx+1, :) - path_dense(lookAheadIdx, :);
        lookAheadPoint = path_dense(lookAheadIdx, :) + (segmentVector / norm(segmentVector)) * remainingDist;
        break; % 찾았으므로 루프 종료
    end

    % 누적 거리 업데이트하고 다음 점으로 이동
    currentDist = currentDist + distToNext;
    lookAheadIdx = lookAheadIdx + 1;

    % 만약 break 없이 루프가 끝까지 돌았다면, 경로 끝점이 look-ahead point
    if lookAheadIdx == size(path_dense, 1)
       lookAheadPoint = path_dense(end, :);
    end
end

% 만약 closestIdx가 이미 마지막 점이면 lookAheadPoint는 마지막 점
if closestIdx == size(path_dense, 1)
    lookAheadPoint = path_dense(end, :);
end


%% 5. 시각화
figure;
hold on; grid on; axis equal;

% 촘촘한 경로 플롯 (파란색 실선)
plot(path_dense(:,1), path_dense(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Dense Path');

% 원본 Waypoint 플롯 (검은색 마커)
plot(path_original(:,1), path_original(:,2), 'kd', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'Original Waypoints');

% 로봇 위치 및 방향 플롯 (빨간색 원 및 화살표)
plot(robotX, robotY, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Robot Position');
quiver(robotX, robotY, cos(robotTheta)*0.5, sin(robotTheta)*0.5, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off', 'DisplayName', 'Robot Orientation'); % 방향 화살표

% 가장 가까운 점 플롯 (녹색 사각형)
plot(closestPoint(1), closestPoint(2), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Closest Point');

% Look-ahead Point 플롯 (마젠타색 별)
plot(lookAheadPoint(1), lookAheadPoint(2), 'mp', 'MarkerSize', 12, 'MarkerFaceColor', 'm', 'DisplayName', 'Look-ahead Point');

% 로봇 -> 가장 가까운 점 연결선 (녹색 점선)
plot([robotX, closestPoint(1)], [robotY, closestPoint(2)], 'g--', 'LineWidth', 1, 'HandleVisibility','off'); % 범례에는 표시 안 함

% 로봇 -> Look-ahead Point 연결선 (마젠타색 점선) - 로봇이 향해야 할 방향 시각화
plot([robotX, lookAheadPoint(1)], [robotY, lookAheadPoint(2)], 'm--', 'LineWidth', 1.5, 'DisplayName', 'Direction to Look-ahead');

% Look-ahead Distance 표시 (경로 상에 표시하기는 복잡하므로, 참고용 텍스트 추가)
text(closestPoint(1) + 0.1, closestPoint(2) + 0.1, sprintf('d* = %.1f', lookAheadDist), 'Color', 'k', 'FontSize', 10);

% 꾸미기
title('Look-ahead Distance Concept Visualization');
xlabel('X (m)');
ylabel('Y (m)');
legend('show', 'Location', 'northwest');
xlim([min(path_original(:,1))-1, max(path_original(:,1))+1]);
ylim([min(path_original(:,2))-1, max(path_original(:,2))+1]);

hold off;
