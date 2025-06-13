function config = performance_config()
%PERFORMANCE_CONFIG 성능 최적화 설정
%
% Returns:
%   config - 성능 최적화 설정 구조체
%
% 설정 내용:
%   - 메모리 최적화 설정
%   - 병렬 처리 최적화
%   - 프로파일링 설정

% ========== 메모리 최적화 ==========
config.memory.preallocation = true;     % 배열 사전 할당 활성화
config.memory.buffer_size = 200;        % 버퍼 크기
config.memory.safety_factor = 1.2;      % 안전 여유 비율

% ========== 병렬 처리 최적화 ==========
config.parallel.broadcast_opt = true;   % Broadcast Variable 최적화
config.parallel.local_vars = true;      % 로컬 변수 추출 활성화
config.parallel.chunk_size = 50;        % 청크 크기

% ========== 프로파일링 설정 ==========
config.profiling.enabled = true;        % 프로파일링 활성화
config.profiling.memory_monitor = true; % 메모리 모니터링
config.profiling.time_measure = true;   % 실행 시간 측정

% ========== 캐시 설정 ==========
config.cache.enabled = true;            % 캐시 활성화
config.cache.max_size = 1000;          % 최대 캐시 크기
config.cache.cleanup_threshold = 0.8;   % 정리 임계값

% ========== 디버깅 설정 ==========
config.debug.logging.enabled = false;   % Transform 디버깅 완료 후 비활성화 (성능 고려)
config.debug.logging.transform = true;  % Transform 관련 로깅
config.debug.logging.timing = true;     % 시간 관련 로깅
config.debug.verbose = false;           % 상세 로깅 모드

end
