clc;
clear;
close all;

% --- 1. Environment Setup and PRM Path Generation ---
disp('Loading map and generating PRM path...');
load exampleMaps
map = binaryOccupancyMap(complexMap);

% Inflate map for collision avoidance
mapInflated = copy(map);
RobotWidth = 1.0; % Approximate robot radius or half-width
inflate(mapInflated, RobotWidth);

% PRM parameters
prm = robotics.PRM(mapInflated);
prm.NumNodes = 500;
prm.ConnectionDistance = 10; % Max distance between connected nodes

% Define start and end locations
startLocation = [4.0 2.0];
endLocation = [48.0 35.0];

% Find path using PRM
path = findpath(prm, startLocation, endLocation);

if isempty(path)
    error('PRM failed to find a path. Try increasing NumNodes or ConnectionDistance.');
end

disp('PRM path generated.');

% --- 2. Refine Path (Dense Trajectory) ---
disp('Refining the PRM path into a dense trajectory...');
samplingDistance = 0.15; % Desired spacing between points on the dense path (m)
ref_traj = path(1,:); % Start with the first waypoint

for i = 1:(size(path, 1) - 1)
    startPoint = path(i,:);
    endPoint = path(i+1,:);

    segmentVector = endPoint - startPoint;
    segmentLength = norm(segmentVector);

    % Interpolate if segment is longer than sampling distance
    if segmentLength > samplingDistance
        numPoints = round(segmentLength / samplingDistance) + 1;
        x_dense = linspace(startPoint(1), endPoint(1), numPoints);
        y_dense = linspace(startPoint(2), endPoint(2), numPoints);

        % Append new points (excluding the first one, it's the end of the previous segment)
        ref_traj = [ref_traj; [x_dense(2:end)', y_dense(2:end)']];
    else
        % If segment is short, just add the endpoint
        ref_traj = [ref_traj; endPoint];
    end
end
disp(['Dense trajectory created with ', num2str(size(ref_traj, 1)), ' points.']);


% --- 3. Pure Pursuit Controller Setup ---
disp('Initializing Pure Pursuit controller and simulation parameters...');
% Robot Initial State
robotInitialPose = [startLocation, 0]'; % [x; y; theta]
robotCurrentPose = robotInitialPose;
robotGoal = endLocation(:); % Ensure robotGoal is a column vector

% Simulation Parameters
sampleTime = 0.1;          % Simulation time step (s)
RobotMaxVel = 1.5;         % Robot maximum linear velocity (m/s)
RobotMaxOmega = pi;        % Robot maximum angular velocity (rad/s) - Adjust as needed
goalRadius = 0.5;          % Radius around the goal to consider 'reached' (m)

% Dynamic Parameter Gains (튜닝 필요)
% For Dynamic Look-ahead Distance: LAD = Kl_lad * v_cmd + L_min
Kl_lad = 0.5;              % Proportional gain for LAD based on velocity (s)
L_min = 0.3;               % Minimum Look-ahead Distance (m)
L_max = 3.0;               % Maximum Look-ahead Distance (m) - Optional limit

% For Dynamic Velocity: v_cmd = RobotMaxVel * max(min_ratio, 1 - K_curve * abs(error_th))
K_curve = 0.4;             % Gain for reducing speed based on heading error (rad^-1)
min_vel_ratio = 0.1;       % Minimum velocity as a fraction of RobotMaxVel (e.g., 0.1 means 10%)
min_vel = RobotMaxVel * min_vel_ratio; % Minimum velocity (m/s)

% Initialize dynamic variables
v_cmd = min_vel; % Start with minimum velocity
LookAheadDist = L_min;

% Visualization setup
figure(1)
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('Pure Pursuit Simulation on Complex Map');

% Show Map and PRM
show(map)   % Show original map
show(prm);  % Show PRM nodes and connections
% plot(path(:,1), path(:,2), 'g-', 'LineWidth', 2); % Plot original PRM path (green)
plot(ref_traj(:,1), ref_traj(:,2), 'b:', 'LineWidth', 1); % Plot dense trajectory (blue dotted)

% History for plotting robot trajectory
robotTrajectoryHistory = robotCurrentPose(1:2)';

% Rate control for visualization speed
vizRate = rateControl(1/sampleTime);

disp('Simulation starting...');
% --- 4. Simulation Loop ---
distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);

while( distanceToGoal > goalRadius )

    % --- Pure Pursuit Logic ---
    % a. Calculate Dynamic Look-ahead Distance based on current velocity
    LookAheadDist = Kl_lad * v_cmd + L_min;
    LookAheadDist = max(L_min, min(L_max, LookAheadDist)); % Apply limits

    % b. Find Look-ahead Point
    %   b1. Find the closest point on the dense trajectory to the robot
    [~, closestIdx] = min(sum((ref_traj - robotCurrentPose(1:2)').^2, 2));

    %   b2. Search forward from the closest point to find the look-ahead point
    distAccumulated = 0;
    lookAheadIdx = closestIdx;
    while distAccumulated < LookAheadDist && lookAheadIdx < size(ref_traj, 1)
        distSegment = norm(ref_traj(lookAheadIdx+1,:) - ref_traj(lookAheadIdx,:));
        if distAccumulated + distSegment > LookAheadDist
            % Interpolate to find the exact point if needed, or just take the point
            % For simplicity, we take the point just before exceeding LAD or the one at exceeding index.
            % Let's take the point at lookAheadIdx+1 as it crossed the threshold.
            lookAheadIdx = lookAheadIdx + 1;
            break; % Found point roughly LAD away
        end
        distAccumulated = distAccumulated + distSegment;
        lookAheadIdx = lookAheadIdx + 1;
    end
    lookAheadPoint = ref_traj(lookAheadIdx, :)'; % Target point [x*; y*]

    % c. Calculate Heading Error
    theta_desired = atan2(lookAheadPoint(2) - robotCurrentPose(2), ...
                          lookAheadPoint(1) - robotCurrentPose(1));
    error_th = angdiff(robotCurrentPose(3), theta_desired);

    % d. Calculate Dynamic Linear Velocity based on heading error
    %    Reduce speed significantly for large turns
    speed_reduction_factor = max(min_vel_ratio, 1 - K_curve * abs(error_th));
    v_cmd_raw = RobotMaxVel * speed_reduction_factor;
    v_cmd = max(min_vel, min(RobotMaxVel, v_cmd_raw)); % Apply limits [min_vel, RobotMaxVel]

    % e. Calculate Angular Velocity (Pure Pursuit Formula)
    %    Use the *actual* distance to the lookAheadPoint, or the calculated LookAheadDist.
    %    Using LookAheadDist (d*) is standard.
    w_cmd_raw = (2 * v_cmd * sin(error_th)) / LookAheadDist;
    w_cmd = max(-RobotMaxOmega, min(RobotMaxOmega, w_cmd_raw)); % Apply limits

    % --- Robot Kinematics ---
    % f. Update Robot Pose
    currentVel = [v_cmd * cos(robotCurrentPose(3)); ...
                  v_cmd * sin(robotCurrentPose(3)); ...
                  w_cmd];
    robotCurrentPose = robotCurrentPose + currentVel * sampleTime;

    % g. Store Robot Trajectory
    robotTrajectoryHistory = [robotTrajectoryHistory; robotCurrentPose(1:2)'];

    % --- Visualization Update ---
    % h. Redraw simulation elements
    % hold off; % Clear previous robot drawing
    show(map) % Redraw map
    show(prm); % Redraw PRM
    % plot(path(:,1), path(:,2), 'g-', 'LineWidth', 0.5); % Original PRM path
    plot(ref_traj(:,1), ref_traj(:,2), 'b:', 'LineWidth', 0.5); % Dense Trajectory
    plot(robotTrajectoryHistory(:,1), robotTrajectoryHistory(:,2), 'r-', 'LineWidth', 0.5); % Robot's path
    hold on; grid on; axis equal; % Prepare for robot plot

    % Plot the current robot pose
    plotTransforms([robotCurrentPose(1:2)', 0], ...
                   axang2quat([0 0 1 robotCurrentPose(3)]), ...
                   "MeshFilePath", "groundvehicle.stl", ...
                   "Parent", gca, "View","2D", "FrameSize", 0.5);

    % Plot look-ahead point (optional visualization)
    % plot(lookAheadPoint(1), lookAheadPoint(2), 'm*', 'MarkerSize', 8);

    % Update title with current info (optional)
    title(sprintf('Pure Pursuit Sim - V: %.2f m/s, W: %.2f rad/s, LAD: %.2f m', v_cmd, w_cmd, LookAheadDist));

    xlim([0 map.XWorldLimits(2)]); % Adjust limits if needed
    ylim([0 map.YWorldLimits(2)]);
    xlabel('X (m)'); ylabel('Y (m)');

    waitfor(vizRate); % Control simulation speed

    % i. Update Distance to Goal for loop condition
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);

end

disp('Goal reached!');
hold off;

% --- End of Simulation ---