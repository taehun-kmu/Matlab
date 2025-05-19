clear; clc; 

dataDir = fullfile(pwd, '../Data/zip'); % Working Directory
RawdataDir = fullfile(pwd, '../Data/unzip');

if ~exist(RawdataDir, 'dir')
    mkdir(RawdataDir);
    fprintf('Created directory: %s\n', RawdataDir);
end

filelist = {
    'train-images-idx3-ubyte.gz', ...
    'train-labels-idx1-ubyte.gz', ...
    't10k-images-idx3-ubyte.gz', ...
    't10k-labels-idx1-ubyte.gz'
};

fprintf('Starting unzipping process...\n');
for i = 1:length(filelist)
    gzFile = fullfile(dataDir, filelist{i});
    if exist(gzFile, 'file')
        fprintf('Unzipping %s to %s\n', gzFile, RawdataDir);
        gunzip(gzFile, RawdataDir);
    else
        warning('File not found, skipping: %s', gzFile);
    end
end
fprintf('Unzipping process finished.\n');

% Unzip & File loading
XTrain = processImagesMNIST(fullfile(RawdataDir, 'train-images-idx3-ubyte'));
YTrain = processLabelsMNIST(fullfile(RawdataDir, 'train-labels-idx1-ubyte'));
XTest  = processImagesMNIST(fullfile(RawdataDir, 't10k-images-idx3-ubyte'));
YTest  = processLabelsMNIST(fullfile(RawdataDir, 't10k-labels-idx1-ubyte'));

% 학습 데이터 개수 조절 (원하는 개수로 변경)
% numTrainSamples = 10000;
% XTrain = XTrain(:, :, :, 1:numTrainSamples);
% YTrain = YTrain(1:numTrainSamples);

% Fashion-MNIST Class Name
classNames = ["T-shirt/top", "Trouser", "Pullover", "Dress", "Coat", "Sandal", "Shirt", "Sneaker", "Bag", "Ankle boot"];

% Print Image
figure(1)
tiledlayout("flow");
numToShow = 20;

perm = randperm(size(XTrain, 4), numToShow);

for i = 1:numToShow
    img = XTrain(:, :, 1, perm(i));   % 28x28 image
    labelIndex = YTrain(perm(i));      % categorical label (number)
    labelName = classNames(double(labelIndex)); % Convert Number to Class name

    nexttile
    imshow(img, [])
    title(labelName, 'FontSize', 10) % Show class name as title
end

%% 2. Data Augmentation
pixelRange = [-15 15];
scaleRange = [0.85 1.15];
rotationRange = [-10 10];

imageAugmenter = imageDataAugmenter( ...
    'RandXReflection', true, ...
    'RandXTranslation', pixelRange, ...
    'RandYTranslation', pixelRange, ...
    'RandRotation', rotationRange, ...
    'RandScale', scaleRange);

augimdsTrain = augmentedImageDatastore([28 28 1], XTrain, YTrain, ...
    'DataAugmentation', imageAugmenter, ...
    'ColorPreprocessing', 'none');

%% 3. Defining the CNN structure
layers = [
    imageInputLayer([28 28 1], 'Name', 'input', 'Normalization', 'none')

    % Block 1
    convolution2dLayer(3, 32, 'Padding', 'same', 'Name', 'conv1_1')
    batchNormalizationLayer('Name', 'bn1_1')
    reluLayer('Name', 'relu1_1')
    convolution2dLayer(3, 32, 'Padding', 'same', 'Name', 'conv1_2')
    batchNormalizationLayer('Name', 'bn1_2')
    reluLayer('Name', 'relu1_2')
    maxPooling2dLayer(2, 'Stride', 2, 'Name', 'pool1')
    dropoutLayer(0.25, 'Name', 'drop1')
    
    % Block 2
    convolution2dLayer(3, 64, 'Padding', 'same', 'Name', 'conv2_1')
    batchNormalizationLayer('Name', 'bn2_1') 
    reluLayer('Name', 'relu2_1')
    convolution2dLayer(3, 64, 'Padding', 'same', 'Name', 'conv2_2')
    batchNormalizationLayer('Name', 'bn2_2')
    reluLayer('Name', 'relu2_2')
    maxPooling2dLayer(2, 'Stride', 2, 'Name', 'pool2')
    dropoutLayer(0.25, 'Name', 'drop2')

    % Block 3
    convolution2dLayer(3, 128, 'Padding', 'same', 'Name', 'conv3_1')
    batchNormalizationLayer('Name', 'bn3_1')
    reluLayer('Name', 'relu3_1')
    convolution2dLayer(3, 128, 'Padding', 'same', 'Name', 'conv3_2')
    batchNormalizationLayer('Name', 'bn3_2')
    reluLayer('Name', 'relu3_2')
    maxPooling2dLayer(2, 'Stride', 2, 'Name', 'pool3')
    dropoutLayer(0.5, 'Name', 'drop3')

    % Fully Connected Layers
    fullyConnectedLayer(512, 'Name', 'fc1')
    batchNormalizationLayer('Name', 'bn_fc1')
    reluLayer('Name', 'relu_fc1')
    dropoutLayer(0.5, 'Name', 'drop_fc1')
    
    fullyConnectedLayer(10, 'Name', 'fc_output')
    softmaxLayer('Name', 'softmax')
    classificationLayer('Name', 'output')];

analyzeNetwork(layers);

%% 4. Learning options
miniBatchSize = 128;
maxEpochs = 80;
initialLearnRate = 0.001;
l2Regularization = 0.0001;
validationFrequency = floor(size(XTrain, 4)/miniBatchSize);
validationPatience = 10;

options = trainingOptions('adam', ...
    'InitialLearnRate', initialLearnRate, ...
    'MaxEpochs', maxEpochs, ...
    'MiniBatchSize', miniBatchSize, ...
    'Shuffle', 'every-epoch', ...
    'ValidationData', {XTest, YTest}, ...
    'ValidationFrequency', validationFrequency, ...
    'ValidationPatience', validationPatience, ...
    'LearnRateSchedule', 'piecewise', ...
    'LearnRateDropFactor', 0.1, ...
    'LearnRateDropPeriod', 30, ...
    'L2Regularization', l2Regularization, ...
    'Plots', 'training-progress', ...
    'Verbose', true, ...
    'ExecutionEnvironment', 'auto');

%% 5. Train Network
fprintf('Starting network training...\n');
[net, trainInfo] = trainNetwork(augimdsTrain, layers, options);

%% 6. Evaluate the network performance
% 훈련 세트에서의 정확도 계산
YPred = classify(net, XTrain);
trainAccuracy = sum(YPred == YTrain)/numel(YTrain);
fprintf('Training Accuracy: %.2f%%\n', trainAccuracy * 100);

% 검증 세트에서의 정확도 계산
YPredTest = classify(net, XTest);
validationAccuracy = sum(YPredTest == YTest)/numel(YTest);
fprintf('Validation Accuracy: %.2f%%\n', validationAccuracy * 100);

% 정확도를 그래프로 시각화
figure(2);
bar([trainAccuracy*100, validationAccuracy*100]);
ylim([0 100]);
xticks([1 2]);
xticklabels({'Train', 'Validation'});
title('Model Performance Comparison');
ylabel('Accuracy (%)');
grid on;
text(1, trainAccuracy*100+3, [num2str(trainAccuracy*100, '%.2f') '%'], 'HorizontalAlignment', 'center');
text(2, validationAccuracy*100+3, [num2str(validationAccuracy*100, '%.2f') '%'], 'HorizontalAlignment', 'center');

% 혼동 행렬 시각화 (검증 세트)
figure(3);
confusionchart(YTest, YPredTest, 'RowSummary', 'row-normalized', 'Title', 'Confusion Chart for Validation Data');


%% ==========  Auxiliary Functions ==========

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