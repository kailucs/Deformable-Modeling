clc;
clear;
% %% Save probedPointsWithN, used for removeCameraBias.m
% DATAFOLDER = 'data/';
% probedPointsWithN = dlmread(strcat(DATAFOLDER,'probePcd.txt'),' ');
% probedPointsWithN = probedPointsWithN(:,1:10);
% save('probedWithN.mat','probedPointsWithN')

load('originalData.mat','testing20','testing4','training20','training4')

%DATAFOLDER = 'data/';
DATAFOLDER = 'data_bias_removed/';
pcd = dlmread(strcat(DATAFOLDER,'originalPcd.txt'),' ');
minMaxDisp = 5;
probedPoints = dlmread(strcat(DATAFOLDER,'probePcd.txt'),' ');
probedPoints = probedPoints(:,[1:6 10]);
[m,n] = size(probedPoints);
data =[];
for i=1:1:m
    point = probedPoints(i,:);
    %load force data
    %python is zero based
    forceData = dlmread(strcat(DATAFOLDER,'force',num2str(i-1),'.txt'),' ');
    forceData = forceData(:,[2 1]);
    [forceDataN,~] = size(forceData);
    A = repmat(point,forceDataN,1);
    B =[A forceData];
    data = [data;B];
    if forceData(end,1) < minMaxDisp
        minMaxDisp = forceData(end,1);
    end
end

trainingData4 = [];
trainingData20 = [];

for i=1:1:m
    tmp = find(training4 == i);
    if norm(tmp,1) > 0.01
        point = probedPoints(i,:);
        %load force data
        %python is zero based
        forceData = dlmread(strcat(DATAFOLDER,'force',num2str(i-1),'.txt'),' ');
        forceData = forceData(:,[2 1]);
        [forceDataN,~] = size(forceData);
        A = repmat(point,forceDataN,1);
        B =[A forceData];
        trainingData4 = [trainingData4;B];
    end
end

for i=1:1:m
    tmp = find(training20 == i);
    if norm(tmp,1) > 0.01
        point = probedPoints(i,:);
        %load force data
        %python is zero based
        forceData = dlmread(strcat(DATAFOLDER,'force',num2str(i-1),'.txt'),' ');
        forceData = forceData(:,[2 1]);
        [forceDataN,~] = size(forceData);
        A = repmat(point,forceDataN,1);
        B =[A forceData];
        trainingData20 = [trainingData20;B];
    end
end

%% get the scale of the data
columnMax = max(data);
columnMin = min(data);
dataRange = columnMax - columnMin;
%% offset and scale data
%offset the data..
data = modifyData(data,dataRange,columnMin);
trainingData4 = modifyData(trainingData4,dataRange,columnMin);
trainingData20 = modifyData(trainingData20,dataRange,columnMin);

bias = columnMin;
scale = dataRange;
save('correctedData.mat');%,'data','pcd','probedPoints','bias','scale')