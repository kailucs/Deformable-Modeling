clc;clear;
%This files load all the point and linear data.. not the cylinder data
%2 is the panda..
DATAFOLDER = 'data/exp_4/';
%% Point Cloud
%pcd:[position(3) color(3) normal(3) curvature(1)]
pcd = dlmread(strcat(DATAFOLDER,'originalPcd.txt'),' ');
%% Point Data
%Data matrix:[position(3) color(3) curvature(1) displacement(1) force(1)]
%probedPoint = [position(3) color(3) normal(3) curvature(1)]
%minMaxDisp = 5;
probedPoints = dlmread(strcat(DATAFOLDER,'probePcd.txt'),' ');
probedPoints = probedPoints(:,(1:10));
[m,n] = size(probedPoints);
pointData =[];
for i=1:1:m
    point = probedPoints(i,[1:6 10]); %learning data do not include normal
    %load force data
    %python is zero based
    i
    forceData = dlmread(strcat(DATAFOLDER,'point/','force_',num2str(i-1),'.txt'),' ');
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
%need to format the data into a struct
%The linear probing data is a struct
%NofPoints:integer specifying the total numebr of collected pts
%field: -lineStarts is a M*3 matrix, where each row is the position of the
%line starting point
%-lineEnds is a M*3 matrix
%-lineNormals is a M*3 matrix
%-lineTorqueAxes is a M*3 matrix
%-data is a N*M*8? matrix, similar to that in the point case
%[color, k,d,f,tau]
%-startEndIndex M*2


lineData = {};


lineData.NofPoints = size(pcd,1);
%probedPointsLine:[position color normal k lineDirection]
probedPointsLine = dlmread(strcat(DATAFOLDER,'probePcd_line_theta.txt'),' ');
probedPointsLine = probedPointsLine(:,(1:13));
probeLength = 0.05;
lineEnds = [];lineStarts =[];lineNormals=[];lineTorqueAxes = [];

for i=1:1:size(probedPointsLine,1)
    %position,direction
    position = probedPointsLine(i,1:3);
    direction = probedPointsLine(i,11:13);
    direction = direction/norm(direction);
    normal = probedPointsLine(i,7:9);
    normal = normal/norm(normal);
    axis = cross(direction,normal); %%%%might need to change this...
    axis = axis/norm(axis);
    lineStarts = [lineStarts;position-0.5*direction*probeLength];
    lineEnds = [lineEnds;position+0.5*direction*probeLength];
    lineNormals = [lineNormals;normal];
    lineTorqueAxes = [lineTorqueAxes;axis];
end
lineData.lineStarts = lineStarts;
lineData.lineEnds = lineEnds;
lineData.lineNormals = lineNormals;
lineData.lineTorqueAxes = lineTorqueAxes;

%lineData.data have [position color curvature d f tau]
data =[]; %to lazy to preallocate space..
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
lineData.data = data;
lineData.startEndIndex = startEndIndex;
%% Generate Training and Testing POINT data 
fullIndex = 1:1:size(probedPoints,1);
%%% randomly sample N locations
size(probedPoints,1)
%N = floor(size(probedPoints,1)/2)
N = size(probedPoints,1);
trainingIndex = datasample(fullIndex,N,'Replace',false);
testingIndex = fullIndex;
testingIndex(trainingIndex) = [];
trainingData = [];
%testingData is not created here because we need to know the displacement.

for ele = trainingIndex
        point = probedPoints(ele,[1:6 10]);
        %load force dat
        %python is zero based
        forceData = dlmread(strcat(DATAFOLDER,'point/','force_',num2str(ele-1),'.txt'),' ');
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
save(strcat(DATAFOLDER,'originalData.mat'),'pcd','pointData','probedPoints',...
    'probedPointsLine','bias','scale','lineData','trainingData','trainingIndex',...
'testingIndex')