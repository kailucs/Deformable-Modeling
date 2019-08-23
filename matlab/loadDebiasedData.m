clc;clear;
%This files load all the point and linear data.. not the cylinder data
DATAFOLDER = 'data/exp_1/';
DEBIASEDFOLDER = 'data/exp_1_debiased/';
load(strcat(DATAFOLDER,'originalData.mat'),'pcd','pointData','probedPoints',...
    'probedPointsLine','bias','scale','lineData','trainingData','trainingIndex',...
'testingIndex')
%% Point Cloud
%pcd:[position(3) color(3) normal(3) curvature(1)]
pcd = dlmread(strcat(DEBIASEDFOLDER,'originalPcd.txt'),' ');
%% Point Data
%Data matrix:[position(3) color(3) curvature(1) displacement(1) force(1)]
%probedPoint = [position(3) color(3) normal(3) curvature(1)]
%minMaxDisp = 5;
probedPoints = dlmread(strcat(DEBIASEDFOLDER,'probePcd.txt'),' ');
probedPoints = probedPoints(:,(1:10));
[m,n] = size(probedPoints);
pointData =[];
for i=1:1:m
    point = probedPoints(i,[1:6 10]); %learning data do not include normal
    %load force data
    %python is zero based
    i
    forceData = dlmread(strcat(DEBIASEDFOLDER,'point/','force_',num2str(i-1),'.txt'),' ');
    forceData = forceData(:,[2 1]);
    [forceDataN,~] = size(forceData);
    A = repmat(point,forceDataN,1);
    B =[A forceData];
    pointData = [pointData;B];
    %if forceData(end,1) < minMaxDisp
    %    minMaxDisp = forceData(end,1);
    %end
end
%minMaxDisp

%% Line Data
%use the original line data
data = [];
startEndIndex = [];
for i=1:1:size(probedPoints,1)
    startIndex = size(data,1)+1;
    point = probedPoints(i,:);
    %load force and torque data
    %Each row of force data for line probe include
    %[fx fy fz fn d]
    % similar for torque..
    i
    forceData = dlmread(strcat(DATAFOLDER,'line/','force_',num2str(i-1),'.txt'),' ');
    forceData = forceData(:,[5 4]);
    torqueData = dlmread(strcat(DATAFOLDER,'line/','torque_',num2str(i-1),'.txt'),' ');
    torqueData = torqueData(:,4);
    A = repmat(point,size(forceData,1),1);
    B =[A forceData torqueData];
    data = [data;B];
    endIndex = size(data,1);
    startEndIndex = [startEndIndex;startIndex endIndex];
end
lineData.startEndIndex = startEndIndex;

%% Generate Training and Testing POINT data 
%use the same index as original data
trainingData = [];
for ele = trainingIndex
        point = probedPoints(ele,[1:6 10]);
        %load force dat
        %python is zero based
        forceData = dlmread(strcat(DEBIASEDFOLDER,'point/','force_',num2str(ele-1),'.txt'),' ');
        forceData = forceData(:,[5 4]);
        [forceDataN,~] = size(forceData);
        A = repmat(point,forceDataN,1);
        B =[A forceData];
        trainingData = [trainingData;B];

end

%% get the scale of the data
%Data matrix:[position(3) color(3) curvature(1) displacement(1) force(1)]
columnMax = max(pointData);
columnMin = min(pointData);
dataRange = columnMax - columnMin;
%% offset and scale data
pointData = modifyData(pointData,dataRange,columnMin);
trainingData = modifyData(trainingData,dataRange,columnMin);
bias = columnMin;
scale = dataRange;
save(strcat(DEBIASEDFOLDER,'debiasedData.mat'),'pcd','pointData','probedPoints',...
    'probedPointsLine','bias','scale','lineData','trainingData','testingIndex')
