clc;
clear;
close all;

% Data Load
load('Problem_1/P_Control.mat');
P_robotPath = robotPath;
P_error_history = error_history;
P_time_history_steps = 0:length(P_error_history)-1;
P_time_history_seconds = P_time_history_steps * 0.1;

load('Problem_2/PI_Control.mat');
PI_robotPath = robotPath;
PI_error_history = error_history;
PI_time_history_seconds = time_history(2:end);

figure;
hold on; grid on; axis equal;
plot(P_robotPath(:, 1), P_robotPath(:, 2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'P_Control Path');
plot(PI_robotPath(:, 1), PI_robotPath(:, 2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'PI Control Path');

% Waypoint plot
path_waypoint = [2.00   1.00;
                 4.00   1.00;
                 1.25   1.75;
                 5.25   8.25;
                 7.25   8.75;
                 11.75  10.75;
                 12.00  10.00];
plot(path_waypoint(:, 1), path_waypoint(:, 2), "k--d", 'MarkerSize', 8, 'LineWidth', 1, 'DisplayName', 'Waypoints');
xlabel('X (m)'); ylabel('Y (m)');
title('Path Tracking Comparison: P vs PI Controller');
legend('show', 'Location', 'best');
hold off;

% Heading Error plot
figure;
hold on; grid on;
plot(P_time_history_seconds, P_error_history, 'r-', 'LineWidth', 1.5, 'DisplayName', 'P Control Error');
plot(PI_time_history_seconds, PI_error_history, 'b--', 'LineWidth', 1.5, 'DisplayName', 'PI Control Error');
xlabel('Time (s)'); ylabel('Heading Error (rad)');
title('Heading Error Comparison: P vs PI Controller');
legend('show', 'Location', 'best');
hold off;

fprintf('P Controller Mean Absolute Error: %f rad\n', mean(abs(P_error_history)));
fprintf('PI Controller Mean Absolute Error: %f rad\n', mean(abs(PI_error_history(2:end))));

fprintf('P Controller Max Absolute Error: %f rad\n', max(abs(P_error_history)));
fprintf('PI Controller Max Absolute Error: %f rad\n', max(abs(PI_error_history(2:end))));
