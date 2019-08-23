clc;
clear;

%modelFolder = 'trained_model/';
%modelFolder = 'trained_model_backup/';
modelFolder = 'data/exp_4_debiased/trainedModel/';
saveFolder = 'data/exp_4_debiased/';
load('data/exp_4_debiased/debiasedData.mat');
drawFlag = false;
errorFlag = true;
GIFDelay = 1;
%%
pcd = diluteData(pcd,10);
%NofTrainingPtList = [4,20];
NofTrainingPtList = [20];
typeOfModelList = {'P'};
%typeOfModelList = {'P','PC','PCG'};
%typeOfLearnerList = ['GPR','KNN'];
typeOfLearnerList = {'KRR'};
%dispList =-0.007:0.002:0.005;
dispList =-0.001:0.001:0.01;

GIFCounter = 1;
%% Loop over Different types of Learner
for j=1:1:size(typeOfLearnerList,2)
    typeOfLearner = typeOfLearnerList{j};
    
    switch typeOfLearner 
        case 'GPR'
            modelName1 = 'gprMdl';
        case 'KNN'
            modelName1 = 'knnMdl';
        case 'KRR'
            modelName1 = 'krMdl';
    end
       
    %% Loop over 4 & 20 TraingingPtList
    for i=1:1:size(NofTrainingPtList,2)
        NofTrainingPt = NofTrainingPtList(i);
        if NofTrainingPt == 4
            testingPtList = testing4;
        elseif NofTrainingPt ==20    
            testingPtList = testing20;
        end
        
        modelName2 = strcat(modelName1,'_',num2str(NofTrainingPt));

       %% Loop over Type of model
        for k = 1:1:size(typeOfModelList,2)
            typeOfModel = typeOfModelList{k};
            modelName = strcat(modelName2,'_',typeOfModel,'.mat');
            %% Predicting starts here.....
            %load model.. the name is model
            load(strcat(modelFolder,modelName))
            if drawFlag
                frameCounter = 1;
                im={};
                
            end
            if errorFlag
                RMSEList = [];
            end
            for iter = 1:1:size(dispList,2)
                queryDisplacement = dispList(iter);
                testingQueryPts = probedPoints(testingPtList,:);
                
                allQueryPts = pcd(:,[1:6 10]);
                testingQueryPts = [testingQueryPts repmat(queryDisplacement,size(testingQueryPts,1),1)];
                allQueryPts = [allQueryPts repmat(queryDisplacement,size(allQueryPts,1),1)];
                testingQueryPts = actual2model(testingQueryPts,scale,bias);
                allQueryPts = actual2model(allQueryPts,scale,bias);
                switch typeOfModel
                    case 'P'
                        testingQueryPts(:,4:7) = [];
                        allQueryPts(:,4:7) =[];
                    case 'PC'
                        testingQueryPts(:,7) = [];
                        allQueryPts(:,7) =[];
                    case 'PCG'
                end
                
                %%%load the ground truth
                testingPtsTrue = [];
                for iter2=1:1:size(testingPtList,2)
                    if DATATYPE == 0
                        pt = queryForce(testingPtList(iter2),queryDisplacement);
                    elseif DATATYPE == 1
                        pt = queryForceCorrected(testingPtList(iter2),queryDisplacement);
                    end
                    testingPtsTrue = [testingPtsTrue;pt];
                end
                
                %%%%predict for the testing point and entire object
                %%Predict
                if strcmp(typeOfLearner, 'GPR')
                    
                    [ypredTesting,~,~] = predict(model,testingQueryPts);
                    [ypredAll,~,~] = predict(model,allQueryPts);
                    
                elseif strcmp(typeOfLearner, 'KNN')            
                    ypredTesting = knnPredict(model,5,testingQueryPts);
                    ypredAll = knnPredict(model,5,allQueryPts);
                elseif strcmp(typeOfLearner, 'KRR')
                    ypredTesting = kernelRidgePredict(model,testingQueryPts);
                    ypredAll = kernelRidgePredict(model,allQueryPts);
                end
                ypredTesting;
                predictionError = ypredTesting- testingPtsTrue;
                RMSE(predictionError);
                if errorFlag
                    RMSEList = [RMSEList;RMSE(predictionError)];
                end 
                %% Plotting
                if drawFlag
                    fig = figure;
                    switch typeOfModel
                        case 'P'
                            modelTitleName = '(Position Only)';
                        case 'PC'
                            modelTitleName = '(Position+Color)';
                        case 'PCG'
                            modelTitleName = '(P + C + G)';
                    end
                    %%2D force prediction map
                    %subplot(1,2,1)
                   xRange =[-0.02 0.1];
                    yRange =[-0.01 0.16];
                    colormap(hsv)
                    colorBias = -1.0;
                    colorScale = 4.0;
                    for iter2 = 1:1:size(allQueryPts,1)
                        x = allQueryPts(iter2,1)*max(scale(1:3));
                        y = allQueryPts(iter2,2)*max(scale(1:3));
                        force = (ypredAll(iter2)-colorBias)/colorScale;
                        if force > 1
                            force = 1;
                        elseif force < 0
                            force = 0;
                        end      
                        axis equal
                        axis([xRange yRange])
                        xlabel('x (m)')
                        ylabel('y (m)')
                        title(strcat(typeOfLearner,{' '},num2str(NofTrainingPt),'Locations Force Prediction Map at',{' '},num2str(queryDisplacement*1000),...
                            'mm',{' '}, modelTitleName));
                        plot(x,y,'.','Color',hsv2rgb([force,1,1]),'MarkerSize',10)
                        hold on
                    end
                    maxPrediction = max(ypredAll)
                    hc = colorbar;
                    set(hc,'TickLabels',{'-1.0 N','-0.6 N','-0.2 N','0.2 N','0.6 N','1.0 N',...
                        '1.4 N','1.8 N','2.2 N','2.6 N','3.0 N'})


                    %% 2D force error map
                    %subplot(1,2,2)
%                     xRange =[-0.02 0.1];
%                     yRange =[-0.01 0.16];
%                     colormap(hsv)
%                     colorBias = -2.0;
%                     colorScale = 4.0;
%                     for iter3 = 1:1:size(testingQueryPts,1)
%                         %%undistort..
%                         x = testingQueryPts(iter3,1)*max(scale(1:3));
%                         y = testingQueryPts(iter3,2)*max(scale(1:3));
%                         force = (predictionError(iter3)-colorBias)/colorScale;
%                         if force > 1
%                             force = 1;
%                         elseif force < 0
%                             force = 0;
%                         end      
%                         axis equal
%                         axis([xRange yRange])
%                         xlabel('x (m)')
%                         ylabel('y (m)')
%                         title(strcat(typeOfLearner,{' '},num2str(NofTrainingPt),'Locations Prediction Error Map at',{' '},num2str(queryDisplacement*1000),...
%                             'mm',{' '}, modelTitleName));
%                         plot(x,y,'.','Color',hsv2rgb([force,1,1]),'MarkerSize',10)
%                         hold on
% 
%                     end
%                     hc = colorbar;
%                     set(hc,'TickLabels',{'-2.0 N','-1.6 N','-1.2 N','-0.8 N','-0.4 N','0.0 N',...
%                         '0.4 N','0.8 N','1.2 N','1.6 N','2.0 N'})
                    
                    %%%%% save for GIF stuff %%%%%%%%
                    frame = getframe(fig);
                    im{frameCounter} = frame2im(frame);
                    frameCounter = frameCounter +1 ;
                end                
            end
            
            %generate GIF for the displacements
            if drawFlag
                filename = strcat(saveFolder,num2str(GIFCounter),'.gif');
                GIFCounter = GIFCounter + 1;
                for idx = 1:1:frameCounter-1
                    [A,map] = rgb2ind(im{idx},256);
                    if idx == 1
                        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',GIFDelay);
                    else
                        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',GIFDelay);
                    end                
                end
            end
            if errorFlag
            typeOfModel
            NofTrainingPt
            RMSEList = RMSEList.'
            end
            %% Report Error Values
            if errorFlag
%                 figure 
%                 xRange =[-10 6];
%                 yRange =[0 1];
%                 axis equal
%                 axis([xRange yRange])
%                 xlabel('Displacement (mm)')
%                 ylabel('RMSE (N)')
%                 titleStr = strcat(typeOfLearner,{' '},'Prediction Error vs Displacement',{' '},modelTitleName)
%                 title(titleStr);
%                 plot(dispList*1000,RMSEList,'b-')    
            end
 
        end   
    end
end

