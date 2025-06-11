%% Building Modeling
close all
close all hidden

profile on

simTime = 60;    % in seconds
updateRate = 10;  % in Hz (increased from 5 to 10)
scene = uavScenario('UpdateRate', updateRate, 'StopTime', simTime);

% Floor
addMesh(scene, 'polygon', {[0 0;200 0;200 200;100 200;100 100;0 100], [-1 0]}, [0.3 0.3 0.3]);

% Feature
addMesh(scene, 'polygon', {[25 0;75 0;75 50;25 50], [0 80]}, [0.4660 0.6740 0.1880]);  % Building 1
addMesh(scene, 'polygon', {[112 0;200 0;200 75;112 75], [0 120]}, [0.9290 0.6980 0.1250]);  % Building 2
addMesh(scene, 'polygon', {[0 87;25 87;25 100;0 100], [0 10]}, [0 0.5 0]);                % Generator Room
addMesh(scene, 'polygon', {[125 100;200 100;200 175;125 175], [0 10]}, [0 0.4470 0.7410]);      % Swimming Pool
addMesh(scene, 'polygon', {[0 0;5 0;5 10;0 10], [0 6]}, [0.6350 0.0780 0.1840]);         % Security Room

% 추가 빌딩들 (RRT/RRT* 성능 테스트용)
addMesh(scene, 'polygon', {[10 60;30 60;30 80;10 80], [0 40]}, [0.7 0.7 0.1]);    % Building 3 (New)
addMesh(scene, 'polygon', {[80 10;100 10;100 30;80 30], [0 60]}, [0.1 0.7 0.7]);    % Building 4 (New)
addMesh(scene, 'polygon', {[130 80;150 80;150 95;130 95], [0 30]}, [0.7 0.1 0.7]);    % Building 5 (New)
addMesh(scene, 'polygon', {[105 110;120 110;120 140;105 140], [0 70]}, [0.5 0.5 0.5]); % Building 6 (New)
addMesh(scene, 'polygon', {[170 10;190 10;190 30;170 30], [0 55]}, [0.9 0.5 0.1]);    % Building 7 (New)
addMesh(scene, 'polygon', {[40 10;60 10;60 25;40 25], [0 20]}, [0.2 0.2 0.8]);       % Building 8 (New)


%% Create Mapping Trajectory
% Simplified rectangular flight pattern
x_simple = [0, 200, 200, 0, 0];
y_simple = [0, 0, 200, 200, 0];
num_waypoints = length(x_simple); % 새로운 웨이포인트 개수 (5개)
z_simple = 150 * ones(1, num_waypoints); % 비행 고도는 150으로 유지

waypoints = [x_simple', y_simple', z_simple'];

% Specify the orientation as Euler angles (in radians) and convert the
% Euler angles to a quaternion;
orientation_eul = [0 0 0];
orientation_quate = quaternion(eul2quat(orientation_eul));
orientation_vec = repmat(orientation_quate, num_waypoints, 1);

% Specify time vector
time = 0:(simTime/(num_waypoints-1)):simTime;

% Generate trajectory from the specified waypoints and orientations using
% waypointTrajectory system object. Specify the reference frame as 'ENU'
trajectory = waypointTrajectory('Waypoints', waypoints, 'Orientation', orientation_vec, ...
    'SampleRate', updateRate, 'ReferenceFrame', 'ENU', 'TimeOfArrival', time);

% Specify the initial pose of the UAV
initial_pose = [-20 -20 100 1 0 0 0];


% UAV 플랫폼 설정 
plat = uavPlatform("UAV",scene,"Trajectory",trajectory,"ReferenceFrame","ENU"); 
updateMesh(plat,"quadrotor",{15},[1 0 0],eye(4));

lidarmodel = uavLidarPointCloudGenerator("AzimuthResolution",1.2, ...
    "ElevationLimits",[-90 -20],"ElevationResolution",5, ...
    "MaxRange",200,"UpdateRate",5,"HasOrganizedOutput",true);

lidar = uavSensor("Lidar",plat,lidarmodel,"MountingLocation",[0 0 -1],"MountingAngles",[0 0 0]);


% Simulate Mapping Flight And Map Building
[ax1, plotFrames] = show3D(scene); % UAV 플랫폼이 scene에 추가된 후 show3D 호출
hFig1 = get(ax1, 'Parent'); 
set(hFig1, 'NumberTitle', 'off', 'Name', 'Figure 1: UAV Scene'); 
axis(ax1, 'equal'); % show3D 호출 직후 기본 설정
view(ax1, [-115 20]); % show3D 호출 직후 기본 설정

xlim(ax1, [-30 230]); 
ylim(ax1, [-30 230]); 
zlim(ax1, [0 150]);   
axis(ax1, 'equal'); % xlim/ylim/zlim 변경 후 aspect ratio 유지를 위해 다시 호출
hold(ax1, 'on');     

colormap(ax1, 'jet'); 
ptc = pointCloud(nan(1,1,3));
% 이제 plotFrames.UAV 필드가 존재해야 하며, plotFrames.UAV.Lidar가 scatterplot의 Parent로 사용됩니다.
scatterplot = scatter3(nan,nan,nan,1,[0.3020 0.7451 0.9333], "Parent", plotFrames.UAV.Lidar);
scatterplot.XDataSource = "reshape(ptc.Location(:,:,1), [], 1)";
scatterplot.YDataSource = "reshape(ptc.Location(:,:,2), [], 1)";
scatterplot.ZDataSource = "reshape(ptc.Location(:,:,3), [], 1)";
scatterplot.CDataSource = "reshape(ptc.Location(:,:,3), [], 1) - min(reshape(ptc.Location(:,:,3), [], 1))";
hold(ax1, 'off');    % ax를 ax1로 변경

lidarSampleTime = [];
pt = cell(1,((updateRate*simTime) +1)); 
ptOut = cell(1,((updateRate*simTime) +1)); 

map3D = occupancyMap3D(1);

% Figure 2 (Occupancy Map) 초기화
hFig2 = figure; % 새 Figure 생성
set(hFig2, 'NumberTitle', 'off', 'Name', 'Figure 2: Occupancy Map'); % Figure 이름 설정
ax2 = axes('Parent', hFig2); % Figure 2에 axes 생성
show(map3D, 'Parent', ax2); % 초기 map을 ax2에 표시
xlim(ax2, [-30 230]); 
ylim(ax2, [-30 230]); 
zlim(ax2, [0 150]);   
view(ax2, [-115 20]);
axis(ax2, 'equal');

% 축 고정 설정 (카메라 뷰와 축 범위를 고정)
set(ax1, 'XLimMode', 'manual', 'YLimMode', 'manual', 'ZLimMode', 'manual');
set(ax2, 'XLimMode', 'manual', 'YLimMode', 'manual', 'ZLimMode', 'manual');

setup(scene);

ptIdx = 0;
while scene.IsRunning
    ptIdx = ptIdx + 1;
    % Read the simulated lidar data from the scenario
    [isUpdated,lidarSampleTime,pt{ptIdx}] = read(lidar);

    if isUpdated
        % Get Lidar sensor's pose relative to ENU reference frame.
        sensorPose = getTransform(scene.TransformTree, "ENU","UAV/Lidar",lidarSampleTime);
        % Process the simulated Lidar pointcloud.
        ptc = pt{ptIdx};
        ptOut{ptIdx} = removeInvalidPoints(pt{ptIdx});
        % Construct the occupancy map using Lidar readings.
        insertPointCloud(map3D,[sensorPose(1:3,4)' tform2quat(sensorPose)],ptOut{ptIdx},500);        % Figure 1 업데이트 (ax1 사용)
        % figure(1) 호출 제거
        show3D(scene,"Time",lidarSampleTime,"FastUpdate",true,"Parent",ax1); % Parent를 ax1로

        refreshdata(scatterplot, 'caller'); % scatterplot 객체 명시
        % drawnow limitrate % 루프 끝으로 이동
    end    % Show map building real time (Figure 2 업데이트, ax2 사용)
    % figure(2) 호출 제거
    show(map3D, 'Parent', ax2); % Parent를 ax2로

    advance(scene);
    updateSensors(scene); 
    drawnow('limitrate'); % 모든 Figure의 변경사항을 한 번에 업데이트
end

profile viewer