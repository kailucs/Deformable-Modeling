clc;
clear;


MODE = 0;%learn
%DATATYPE = 0; %originial data
DATATYPE = 1; %corrected data

if DATATYPE == 0
    load('originalData.mat');
elseif DATATYPE ==1
    load('correctedData.mat');
end

destinationFolder = 'trained_model_3/';
diluteFactor = 8;
maxIteration = 40;
%%
if MODE == 0
    tic
    % [position(3) color(3) curvature(1) displacement(1) force(1)]

%% 4 points    
    tmp = diluteData(trainingData4,diluteFactor);
    %%
    
    
    X = tmp(:,[1:3 8]);
    y = tmp(:,9);
    rng default      
    gprMdl_4_P = fitrgp(X,y,'OptimizeHyperparameters','all',...
    'HyperparameterOptimizationOptions',...
    struct('AcquisitionFunctionName','expected-improvement-plus',...
    'UseParallel',true,'MaxObjectiveEvaluations',maxIteration));
    
    save(strcat(destinationFolder,'gprMdl_4_P.mat'),'model')
    
    
    
    %%
    X = tmp(:,[1:7 8]);
    y = tmp(:,9);
    rng default      
    gprMdl_4_PCG = fitrgp(X,y,'OptimizeHyperparameters','all',...
    'HyperparameterOptimizationOptions',...
    struct('AcquisitionFunctionName','expected-improvement-plus',...
    'UseParallel',true,'MaxObjectiveEvaluations',maxIteration));
    
    save(strcat(destinationFolder,'gprMdl_4_PCG.mat'),'model')
    
    
    
    
    %%
    X = tmp(:,[1:6 8]);
    y = tmp(:,9);
    rng default    


    gprMdl_4_PC = fitrgp(X,y,'OptimizeHyperparameters','all',...
        'HyperparameterOptimizationOptions',...
    struct('AcquisitionFunctionName','expected-improvement-plus',...
    'UseParallel',true,'MaxObjectiveEvaluations',maxIteration));
    
    save(strcat(destinationFolder,'gprMdl_4_PC.mat'),'model')
    
%% 20 point
    tmp = diluteData(trainingData20,diluteFactor);
    %%
    X = tmp(:,[1:3 8]);
    y = tmp(:,9);
    rng default    

    %https://www.mathworks.com/help/stats/bayesian-optimization-workflow.html
    %The basis function here is probably used to explicitly model the nonzero
    %mean (refer to the GPML book..)
    gprMdl_20_P = fitrgp(X,y,'OptimizeHyperparameters','all',...
        'HyperparameterOptimizationOptions',...
    struct('AcquisitionFunctionName','expected-improvement-plus',...
    'UseParallel',true,'MaxObjectiveEvaluations',maxIteration));
    save(strcat(destinationFolder,'gprMdl_20_P.mat'),'model')
    
    %%
    X = tmp(:,[1:6 8]);
    y = tmp(:,9);
    rng default    


    gprMdl_20_PC = fitrgp(X,y,'OptimizeHyperparameters','all',...
        'HyperparameterOptimizationOptions',...
    struct('AcquisitionFunctionName','expected-improvement-plus',...
    'UseParallel',true,'MaxObjectiveEvaluations',maxIteration));
    
    save(strcat(destinationFolder,'gprMdl_20_PC.mat'),'model')
    %%
    X = tmp(:,[1:7 8]);
    y = tmp(:,9);
    rng default    


    model = fitrgp(X,y,'OptimizeHyperparameters','all',...
        'HyperparameterOptimizationOptions',...
    struct('AcquisitionFunctionName','expected-improvement-plus',...
    'UseParallel',true,'MaxObjectiveEvaluations',20));
    
    save(strcat(destinationFolder,'gprMdl_20_PCG.mat'),'model')

    
   toc
    
end