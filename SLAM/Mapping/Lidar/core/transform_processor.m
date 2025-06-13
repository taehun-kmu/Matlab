function [transforms, cacheHits] = processTransforms(scene, sampleTimes, transformCache, cacheHits, config)
% Transform 캐싱 및 계산 (SRP: Transform 처리만)

frameCount = length(sampleTimes);
cacheKeys = cell(1, frameCount);
transforms = cell(1, frameCount);
needsComputation = false(1, frameCount);

% 캐시 확인
for i = 1:frameCount
    cacheKeys{i} = sprintf('%.6f', sampleTimes(i));
    if isKey(transformCache, cacheKeys{i})
        transforms{i} = transformCache(cacheKeys{i});
        cacheHits = cacheHits + 1;
    else
        needsComputation(i) = true;
    end
end

% 필요한 Transform 계산
computeIndices = find(needsComputation);
if ~isempty(computeIndices)
    newTransforms = computeTransforms(scene, sampleTimes(computeIndices), config);
    
    % 캐시에 저장
    for j = 1:length(computeIndices)
        idx = computeIndices(j);
        if ~isempty(newTransforms{j})
            transforms{idx} = newTransforms{j};
            transformCache(cacheKeys{idx}) = newTransforms{j};
        else
            transforms{idx} = getFallbackTransform(transformCache, config);
        end
    end
end
end

function transforms = computeTransforms(scene, sampleTimes, config)
% Transform 계산 (SRP: 계산만)

transforms = cell(1, length(sampleTimes));
transformTree = scene.TransformTree;
debugMode = isfield(config, 'debug') && isfield(config.debug, 'logging') && config.debug.logging.enabled;

parfor j = 1:length(sampleTimes)
    try
        sampleTime = sampleTimes(j);
        if ~isfinite(sampleTime) || sampleTime < 0
            if debugMode
                fprintf('Warning: Invalid sample time %.6f at index %d\n', sampleTime, j);
            end
            transforms{j} = [];
            continue;
        end
        
        T = getTransform(transformTree, "ENU", "UAV/Lidar", sampleTime);
        
        if isempty(T) || any(~isfinite(T(:)))
            if debugMode
                fprintf('Warning: Invalid transform returned for time %.6f\n', sampleTime);
            end
            transforms{j} = [];
        else
            transforms{j} = T;
        end
        
    catch ME
        if debugMode
            fprintf('Error: Transform computation failed for time %.6f - %s\n', sampleTimes(j), ME.message);
        end
        transforms{j} = [];
    end
end
end

function T = getFallbackTransform(transformCache, config)
% Fallback Transform 반환 (SRP: Fallback 처리만)

debugMode = isfield(config, 'debug') && isfield(config.debug, 'logging') && config.debug.logging.enabled;

if ~isempty(transformCache.keys)
    cacheValues = transformCache.values;
    for k = length(cacheValues):-1:1
        if ~isempty(cacheValues{k}) && all(isfinite(cacheValues{k}(:)))
            T = cacheValues{k};
            if debugMode
                fprintf('Debug: Using fallback transform\n');
            end
            return;
        end
    end
end

T = eye(4);
if debugMode
    fprintf('Debug: Using identity transform\n');
end
end