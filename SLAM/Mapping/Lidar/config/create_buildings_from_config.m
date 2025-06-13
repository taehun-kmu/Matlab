function create_buildings_from_config(scene, buildings_config)
%CREATE_BUILDINGS_FROM_CONFIG 설정 기반 빌딩 생성
%
% Parameters:
%   scene - UAV 시나리오 객체
%   buildings_config - 빌딩 설정 구조체
%
% 기능:
%   - 모든 구역의 빌딩을 설정 기반으로 생성
%   - 성능 최적화된 벡터화 처리

% ========== 바닥 생성 ==========
% Floor - 완전한 직사각형으로 변경
addMesh(scene, 'polygon', {[0 0;200 0;200 200;0 200], [-1 0]}, [0.3 0.3 0.3]);

% ========== 구역별 빌딩 생성 ==========
zones = {'zone1', 'zone2', 'zone3', 'zone4'};

for zone_idx = 1:length(zones)
    zone_name = zones{zone_idx};
    zone_buildings = buildings_config.(zone_name).buildings;
    
    for building_idx = 1:length(zone_buildings)
        building = zone_buildings{building_idx};
        
        % 빌딩 메시 추가
        addMesh(scene, 'polygon', ...
                {building.coords, building.height}, ...
                building.color);
    end
end

fprintf('✅ A total of %d buildings have been created from the configuration.\n', ...
    calculate_total_buildings(buildings_config));

end

function total = calculate_total_buildings(buildings_config)
%CALCULATE_TOTAL_BUILDINGS 총 빌딩 수 계산
zones = {'zone1', 'zone2', 'zone3', 'zone4'};
total = 0;
for i = 1:length(zones)
    total = total + length(buildings_config.(zones{i}).buildings);
end
end
