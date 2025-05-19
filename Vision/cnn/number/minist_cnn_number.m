clc;
clear;

% Importing image data
% unzip("DigitsData.zip");
dataFolder = "DigitsData";
imds = imageDatastore(dataFolder, IncludeSubfolders=true, LabelSource="foldernames");

% Outputting images
figure(1)
tiledlayout("flow");
perm = randperm(10000, 20);

for i = 1:20
    nexttile
    imshow(imds.Files{perm(i)});
end

classNames = categories(imds.Labels);
labelCount = countEachLabel(imds)

img = readimage(imds, 1);
size(img)

% Specify a dataset for training and a dataset for validation
numTrainFiles = 750;
[imdsTrain,imdsValidation] = splitEachLabel(imds, numTrainFiles, "randomized");

% Defining neural network architecture
layers = [ imageInputLayer([28 28 1]) % Image size 28x28x1

    convolution2dLayer(3, 8, Padding = "same") % Filter size 3x3 / number of filters / padding method
    batchNormalizationLayer % Batch normalization
    reluLayer % Activation Function
    
    maxPooling2dLayer(2, Stride=2) % Max Pooling
    
    convolution2dLayer(3, 16, Padding="same")
    batchNormalizationLayer
    reluLayer
    
    maxPooling2dLayer(2, Stride=2)
    
    convolution2dLayer(3, 32, Padding="same")
    batchNormalizationLayer
    reluLayer
    
    fullyConnectedLayer(10)
    softmaxLayer];

% Specify training options
options = trainingOptions("sgdm", ... 
    InitialLearnRate=0.01, ...
    MaxEpochs=4, ... % Epoch: Training on the entire training dataset
    Shuffle="every-epoch", ... % Shuffle data every epoch
    ValidationData=imdsValidation, ...
    ValidationFrequency=30, ... % Validation data accuracy calculation cycle
    Plots="training-progress", ...
    Metrics="accuracy", ...
    Verbose=false); % Disable verbose output

% Training neural networks
net = trainnet(imdsTrain, layers, "crossentropy", options);

% Calculate accuracy using validation footage
scores = minibatchpredict(net, imdsValidation);
YValidation = scores2label(scores, classNames);

TValidation = imdsValidation.Labels;
accuracy = mean(YValidation == TValidation)