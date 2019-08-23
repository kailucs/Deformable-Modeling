clc;
clear;

%modelFolder = 'trained_model/';
%modelFolder = 'trained_model_backup/';
modelFolder = 'data/exp_4_debiased/trained_model/';
saveFolder = 'data/exp_4_debiased/';
forceFolder = 'data/exp_4_debiased/point/';
load('data/exp_4_debiased/debiasedData.mat');
drawFlag = false;
errorFlag = true;
GIFDelay = 1;
%%
pcd = diluteData(pcd,10);
typeOfModelList = {'P'};
typeOfLearnerList = {'KNN'};
dispList =-0.002:0.001:0.005;
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
        testingPtList = testingIndex;
        NofTrainingPt = 48;
        modelName2 = strcat(modelName1,'_',num2str(NofTrainingPt));
       %% Loop over Type of model
        for k = 1:1:size(typeOfModelList,2)
            typeOfModel = typeOfModelList{k};
            modelName = strcat(modelName2,'_',typeOfModel,'.mat');
            %% Predicting starts here.....
            %load model.. the name is model
            load(strcat(modelFolder,modelName))
            if errorFlag
                RMSEList = [];
            end
            for iter = 1:1:size(dispList,2)
                queryDisplacement = dispList(iter)
                testingQueryPts = probedPoints(testingPtList,[1:6 10]); 
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
                    pt = queryForce(testingPtList(iter2),queryDisplacement,forceFolder);
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
            end
            
            %generate GIF for the displacements
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

