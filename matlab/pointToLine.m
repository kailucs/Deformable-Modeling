clc;clear;

DATAFOLDER = 'data/exp_1_debiased/';
modelFolder = 'data/exp_1_debiased/trained_model/';
modelFile = 'knnMdl_121_P.mat';
load(strcat(DATAFOLDER,'debiasedData.mat'))
load(strcat(modelFolder,modelFile))
DEBUGPROJECTEDPTS = false;
DEBUGDISPLACEDPTS = false;
%%the following are loaded:
%'pcd','pointData','probedPoints','probedPointsLine','bias','scale',
%'lineData','trainingData','trainingIndex','testingIndex'
%%need to format the data into a struct
%The linear probing data is a struct
%NofPoints:integer specifying the total numebr of collected pts
%field: -lineStarts is a M*3 matrix, where each row is the position of the
%line starting point
%-lineEnds is a M*3 matrix
%-lineNormals is a M*3 matrix
%-lineTorqueAxes is a M*3 matrix
%-data is a M*8? 
%-startEndIndex

param = 0.03 ;%for distance kernel
%param = 2000
discretization = 0.003;
%% Iterate through different point locations
for iter =24


pointNumber = iter;

%Do everything in the world coordinate
%units: m & newtons


%%%%%%%Test for one probing action...
lineStart0 = lineData.lineStarts(pointNumber,:);
lineEnd0 = lineData.lineEnds(pointNumber,:);
lineTorqueAxes = lineData.lineTorqueAxes(pointNumber,:);
N = 1 + round(norm(lineStart0- lineEnd0)/discretization)
s = linspace(0,1,N);
lineNormal = lineData.lineNormals(pointNumber,:);
localXinW = (lineEnd0-lineStart0)/norm(lineEnd0-lineStart0);
localYinW = (lineTorqueAxes)/norm(lineTorqueAxes);
lineStartinLocal = [0,0];
lineEndinLocal = [dot(lineEnd0-lineStart0,localXinW),...
    dot(lineEnd0-lineStart0,localYinW)];

%%%%%%first project the pcd to the plane of the line
%The start point is the origin of the plane...
projectedPcd = zeros(size(pcd,1),2); %preallocate and cut later
%the projected Pcd is in local frame....
projectedPcdIdxList = zeros(size(pcd,1),1);%Idx in the original pcd 
NofProjectedPoints = 0;
pcdThatWasProjected = [];
for i=1:1:size(pcd,1)
    p = pcd(i,1:3);
    projectedPt = p-dot((p-lineStart0),lineNormal)*lineNormal; %world origin
    projectedPt2D = projectedPt - lineStart0;%world origin
    projectedPt2DinLocal = [dot(projectedPt2D,localXinW),dot(projectedPt2D,localYinW)];
    %make sure point is in the "box" defined by the line
    if (projectedPt2DinLocal(1)<0.051) && (projectedPt2DinLocal(1) > -0.001)...
           && (projectedPt2DinLocal(2)<0.001) &&(projectedPt2DinLocal(2)>-0.001)
        NofProjectedPoints = NofProjectedPoints + 1;
        projectedPcd(NofProjectedPoints,:) = projectedPt2DinLocal;
        projectedPcdIdxList(NofProjectedPoints,:) = i;
        pcdThatWasProjected = [pcdThatWasProjected;p];
    end
end
projectedPcd(NofProjectedPoints:size(pcd,1),:) = [];%cut the excessive space
projectedPcdIdxList(NofProjectedPoints:size(pcd,1),:) = [];

%%%%%%Find the corresponding point on the surface to the line...
surfacePtsAll = []; %These are the surface pts that will be displaced.
%part of the probe not in contact with the object...
NofN = 3;
for i=1:1:N
    tmp =(lineEnd0-lineStart0)*s(i);%the point on the line, projected 
    linePt = [dot(tmp,localXinW),dot(tmp,localYinW)];
    %onto the line
    Idx = knnsearch(projectedPcd,linePt(1:2),'K',NofN);
    %%%We might end up having duplicated pts...
    %%%We should make sure that the discretization is not too fine..
    %%%or should average a few neighbors
    surfacePt = zeros(1,10);
    for j=1:1:NofN
        %disp('---')
        %pcd(projectedPcdIdxList(Idx(j)),1:3)
        %pcd(Idx(j),7:9)
        %projectedPcd
        surfacePt = surfacePt + pcd(projectedPcdIdxList(Idx(j)),1:10);
    end
    surfacePt = surfacePt/NofN;
    surfacePtsAll = [surfacePtsAll;surfacePt];%position in the global frame..
end

%% Go through a bunch of displacements...
queryDList = [];
totalFinNList = [];
for queryD = -0.003:0.001:0.014
%for queryD = 0.012
    queryD
    queryDList = [queryDList;queryD];
    lineStart = lineStart0 - lineNormal*queryD;
    lineEnd = lineEnd0 - lineNormal*queryD;
    lineCenter = (lineStart+lineEnd)/2;
    localXinW = (lineEnd-lineStart)/norm(lineEnd-lineStart);
    localYinW = (lineTorqueAxes)/norm(lineTorqueAxes);
  
    %% Plot for debugging.....
    if DEBUGPROJECTEDPTS
        figure
        xRange =[-0.05 0.2] + [bias(1) bias(1)];
        yRange =[-0.1 0.18] + [bias(2) bias(2)];
        pcdPlot = diluteData(pcd,8);
        for i = 1:1:size(pcdPlot,1)
            %%pcd is the raw data, not scaled for learning
            x = pcdPlot(i,1);
            y = pcdPlot(i,2);
            color = pcdPlot(i,4:6);
            plot(x,y,'.','Color',color,'MarkerSize',10)
            hold on
        end
        %plot projected Pts
        h4 = plot(pcdThatWasProjected(:,1),pcdThatWasProjected(:,2),'.b',...
            'MarkerSize',10);
        %plot the line
        h2 = plot(lineStart(1),lineStart(2),'.g','MarkerSize',20);
        h3 = plot(lineEnd(1),lineEnd(2),'.g','MarkerSize',20);   
        for i=[pointNumber]
                x = probedPoints(i,1);
                y = probedPoints(i,2);
                color = probedPoints(i,4:6);
                %h1 = plot(x,y,'.','Color',color,'MarkerSize',10);
                h1 = plot(x,y,'.g','MarkerSize',20);
        end
        axis equal
        axis([xRange yRange])
        xlabel('x (m)')
        ylabel('y (m)')
        title('Top View of the Testing Object')
    end
    

    %% calculate the nominal displacements
    surfacePts = []; %These are the surface pts that will be displaced...
    nominalD = []; %Column Vector..
    for i=1:1:N
        linePt = (lineEnd-lineStart)*s(i)+lineStart;
        surfacePt = surfacePtsAll(i,1:3);
        normal = surfacePtsAll(i,7:9);
        nominalDisp = -dot(linePt-surfacePt,normal);
        if nominalDisp > 0
            surfacePts = [surfacePts;surfacePtsAll(i,1:10)];%position in the global frame..
            nominalD = [nominalD;nominalDisp];
        end
    end
    originalNominalD = nominalD;
    %nominalD
   %% Plot surface pts.... 
   if DEBUGDISPLACEDPTS
        figure
        plot3(surfacePts(:,1),surfacePts(:,2),surfacePts(:,3),'.b',...
            [lineStart(1),lineEnd(1),lineCenter(1)],...
            [lineStart(2),lineEnd(2),lineCenter(2)],...
            [lineStart(3),lineEnd(3),lineCenter(3)],'.g','MarkerSize',20)     
            axis equal
            xlabel('x (m)')
            ylabel('y (m)')
            zlabel('z (m)')
    end
    
    
    %% Calculate Actual Displacements
    
    %%%Debugging
    %surfacePts = surfacePts(3:5,:);

    %%%
    
    NofSurfacePts = size(surfacePts,1);
    originalSurfacePts = surfacePts; %keep a record for debugging purposes
    if NofSurfacePts > 0
        negativeDisp = true;
        %iteratively get rid of the negative pts
        while negativeDisp
            NofSurfacePts = size(surfacePts,1);
            K = zeros(NofSurfacePts,NofSurfacePts);
            for i = 1:1:NofSurfacePts
                for j = 1:1:NofSurfacePts
                    K(i,j) = invKernel(surfacePts(i,1:3),surfacePts(j,1:3),param);
                end
            end
            actualD = K\nominalD;
            negativeIndex = actualD < 0;
            if norm(double(negativeIndex),1) > 0
                %get rid of the negative pts
                surfacePts(negativeIndex,:) = [];
                nominalD(negativeIndex,:) = [];
            else
                negativeDisp = false;
            end
        end
        %actualD
        nominalD;
        totalF = [0,0,0]; %row vector
        totalTorque = 0; %units:Nm
        for i=1:1:NofSurfacePts
           %%%2 different ways to predict force...
           %queryPt = actual2model([surfacePts(i,1:7) actualD(i)],scale,bias);
           %force = knnPredict(model,5,queryPt([1:3,8]));
           
           queryPt1 = actual2model([surfacePts(i,1:7) nominalD(i)],scale,bias);
           queryPt2 = actual2model([surfacePts(i,1:7) nominalD(i)-actualD(i)],...
               scale,bias);
           force1 = knnPredict(model,5,queryPt1([1:3,8]));
           force2 = knnPredict(model,5,queryPt2([1:3,8]));
           force = force1-force2;
           
           totalF = totalF + force*surfacePts(i,7:9);
           %linePt =(lineEnd-lineStart)*s(i)+lineStart;
           %Use the point on the line to calculate torque..
           %torque = dot(cross(linePt-lineCenter,force),torqueAxis);
           %totalTorque = totalTorque + torque;
        end
        totalFinN = dot(totalF,lineNormal);
        totalFinNList = [totalFinNList;totalFinN];
        actualD
    else
        totalFinNList = [totalFinNList;0];
    end
end


%Nominal Displacements
% allNominalDisplacement = [];
% displacedSurfacePts = [];
% for iter = 1:1:size(originalSurfacePts,1)
%     distanceVector = [];
%     for i=1:1:size(surfacePts,1)
%        distanceVector = [distanceVector,inv2Kernel(surfacePts(i,1:3),...
%            originalSurfacePts(iter,1:3),param)];
%     end
%     allNominalDisplacement = [allNominalDisplacement;...
%         dot(distanceVector,actualD)];
%     displacedSurfacePts = [displacedSurfacePts;-dot(distanceVector,actualD)*...
%         originalSurfacePts(iter,7:9)+originalSurfacePts(iter,1:3)];
% end

% allNominalDisplacement %calculated from actual D
% originalNominalD %calculated from geometry
% originalNominalD - allNominalDisplacement 
%%%%%% Debugging..
%%%%%% should the equlilibrium and displaced surface points.
% figure
% plot3(surfacePts(:,1),surfacePts(:,2),surfacePts(:,3),'.b',...
%     originalSurfacePts(:,1),originalSurfacePts(:,2),originalSurfacePts(:,3),...
% '-b',[lineStart(1),lineEnd(1),lineCenter(1)],...
% [lineStart(2),lineEnd(2),lineCenter(2)],...
% [lineStart(3),lineEnd(3),lineCenter(3)],'-g',...
% displacedSurfacePts(:,1),displacedSurfacePts(:,2),displacedSurfacePts(:,3),'.r',...
% 'MarkerSize',10)     
%             axis equal
%             xlabel('x (m)')
%             ylabel('y (m)')
%             zlabel('z (m)')
            
%%%%%

%get line data
rawLineData = lineData.data(lineData.startEndIndex(pointNumber,1):...
    lineData.startEndIndex(pointNumber,2),:);
%plot predicted force-displacement data for line probes
figure
plot(queryDList*1000,totalFinNList,'b',rawLineData(:,11)*1000,rawLineData(:,12),'r')
xlabel('Displacement(mm)');
ylabel('Force (N)');
legend('Predicted from point','Ground Truth')
title(strcat('Line contact force at location',' ',num2str(iter)))

end
