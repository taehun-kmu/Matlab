clc; clear;

% 도시 블록의 3D 점유 맵 불러오기 
mapData = load("dMapCityBlock.mat"); 
omap = mapData.omap;

% 장애물이 없는 셀로 간주할 임계값 지정
omap.FreeThreshold = 0.5; 

% 장애물 주변에서 안전하게 작동할 수 있도록 점유 맵을 확장하여 완충 지대 추가
inflate(omap,1)

% 상태 변수의 범위가 지정된 SE(3) state space 객체 생성
ss = stateSpaceSE3([0 220;0 220;0 ... 
    100;inf inf;inf inf;inf inf;inf inf]);

% 생성된 상태공간을 사용하여 3차원 점유 맵 상태 유효성 검사기 생성
% 점유 맵을 state validator 객체에 할당
% 샘플링 거리 간격 지정
sv = validatorOccupancyMap3D(ss, ...
     Map = omap, ...
     ValidationDistance = 0.1);

% 최대 연결 거리를 늘리고 최대 반복 횟수를 줄인 RRT 경로 플래너 생성
% 목표까지의 유클리드 거리가 임계값 1미터 미만인 경우에 경로가 목표에 도달했다고 결정하는 사용자 지정 목표 함수 지정
planner = plannerRRT(ss,sv, ...
          MaxConnectionDistance= 3, ...
          MaxIterations = 3000, ...
          GoalReachedFcn= @(~,s,g)(norm(s(1:3)-g(1:3))<1), ...
          GoalBias = 0.1);

planner_star = plannerRRTStar(ss,sv, ...
        ContinueAfterGoalReached=true,...
          MaxConnectionDistance= 3, ...
          MaxIterations = 3000, ...
          GoalReachedFcn= @(~,s,g)(norm(s(1:3)-g(1:3))<1), ...
          GoalBias = 0.1);

% 출발 자세와 목표 자세를 지정합니다.
start = [40 180 25 0.7 0.2 0 0.1];
goal = [150 33 35 0.3 0 0.1 0.6];

% 반복 가능한 결과를 위해 난수 생성기 구성
rng(1,"twister");

% 경로를 계획합니다.
[pthObj1,solnInfo1] = plan(planner,start,goal);

figure(1)
% 계획된 경로 시각화
show(omap)
axis equal
view([-10 55])
hold on
% Start state
scatter3(start(1,1),start(1,2),start(1,3),"g","filled")
% Goal state
scatter3(goal(1,1),goal(1,2),goal(1,3),"r","filled")
% Path
plot3(solnInfo1.TreeData(:,1),solnInfo1.TreeData(:,2),solnInfo1.TreeData(:,3),'yellow.-', LineWidth=2)
plot3(pthObj1.States(:,1),pthObj1.States(:,2),pthObj1.States(:,3), "r-",LineWidth=2)

path_state1 = pthObj1.States;
position1 = path_state1(:,1:3);
diffs1 = diff(position1);
seg_path1 = vecnorm(diffs1,2,2);
RRT_length = sum(seg_path1)

% 
% % RRT-Star
% % 경로를 계획합니다.
[pthObj2,solnInfo2] = plan(planner_star,start,goal);

figure(2)
% 계획된 경로 시각화
show(omap)
axis equal
view([-10 55])
hold on
% Start state
scatter3(start(1,1),start(1,2),start(1,3),"g","filled")
% Goal state
scatter3(goal(1,1),goal(1,2),goal(1,3),"r","filled")
% Path
plot3(solnInfo2.TreeData(:,1),solnInfo2.TreeData(:,2),solnInfo2.TreeData(:,3),'yellow.-', LineWidth=2)
plot3(pthObj2.States(:,1),pthObj2.States(:,2),pthObj2.States(:,3), "r-",LineWidth=2)

path_state2 = pthObj2.States;
position2 = path_state2(:,1:3);
diffs2 = diff(position2);
seg_path2 = vecnorm(diffs2,2,2);
RRTstar_length = sum(seg_path2)