clear;
clc; 


dataFolder = fullfile(pwd); % 현재 작업 폴더
gunzip(fullfile(dataFolder, 'train-images-idx3-ubyte.gz'), dataFolder);
gunzip(fullfile(dataFolder, 'train-labels-idx1-ubyte.gz'), dataFolder);
gunzip(fullfile(dataFolder, 't10k-images-idx3-ubyte.gz'), dataFolder);
gunzip(fullfile(dataFolder, 't10k-labels-idx1-ubyte.gz'), dataFolder);

% 압축 해제 후 파일 로딩
XTrain = processImagesMNIST(fullfile(dataFolder,'train-images-idx3-ubyte'));
YTrain = processLabelsMNIST(fullfile(dataFolder,'train-labels-idx1-ubyte'));
XTest  = processImagesMNIST(fullfile(dataFolder,'t10k-images-idx3-ubyte'));
YTest  = processLabelsMNIST(fullfile(dataFolder,'t10k-labels-idx1-ubyte'));

% Fashion-MNIST 클래스 이름
classNames = ["T-shirt/top", "Trouser", "Pullover", "Dress", "Coat", "Sandal", "Shirt", "Sneaker", "Bag", "Ankle boot"];

% 이미지 출력
figure(1)
tiledlayout("flow");
numToShow = 20;

perm = randperm(size(XTrain, 4), numToShow);

for i = 1:numToShow
    img = XTrain(:, :, 1, perm(i));   % 28x28 이미지
    labelIndex = YTrain(perm(i));      % categorical 라벨 (숫자)
    labelName = classNames(double(labelIndex)); % 숫자 → 클래스 이름으로 변환

    nexttile
    imshow(img, [])
    title(labelName, 'FontSize', 10) % 클래스 이름을 title로 표시
end

    %% 2. CNN 구조 정의 (작성 필요)

    %% 3. 학습 옵션
    % options = trainingOptions('sgdm', ...
    % 'InitialLearnRate', 0.01, ...
    % 'MaxEpochs',30, ...
    % 'MiniBatchSize', 128, ...
    % 'Shuffle', 'every-epoch', ...
    % 'ValidationData',{XTest, YTest}, ...
    % 'ValidationFrequency',30, ...
    % 'Plots','training-progress', ...
    % 'Verbose',false);

    %% 4. 학습 실행
    % net = trainNetwork(XTrain, YTrain, layers, options);


%% ========== 보조 함수들 ==========

function images = processImagesMNIST(filename)
    fid = fopen(filename, 'rb');
    fread(fid, 1, 'int32', 0, 'ieee-be'); % magic number
    numImages = fread(fid, 1, 'int32', 0, 'ieee-be');
    numRows   = fread(fid, 1, 'int32', 0, 'ieee-be');
    numCols   = fread(fid, 1, 'int32', 0, 'ieee-be');
    images = fread(fid, inf, 'unsigned char');
    fclose(fid);

    images = reshape(images, numCols, numRows, 1, numImages);
    images = permute(images, [2 1 3 4]);
    images = im2single(images);
end

function labels = processLabelsMNIST(filename)
    fid = fopen(filename, 'rb');
    fread(fid, 1, 'int32', 0, 'ieee-be'); % magic number
    fread(fid, 1, 'int32', 0, 'ieee-be'); % number of labels
    labels = fread(fid, inf, 'unsigned char');
    fclose(fid);

    labels = categorical(labels);
end