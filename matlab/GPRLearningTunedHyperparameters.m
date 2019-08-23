clc;
clear;


MODE = 0;%learn
DATATYPE = 0; %originial data
%DATATYPE = 1; %corrected data

if DATATYPE == 0
    load('originalData.mat');
elseif DATATYPE ==1
    load('correctedData.mat');
end

destinationFolder = 'trained_model/';
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
    model = fitrgp(X,y,'Sigma',0.067979,'BasisFunction','linear',...
        'KernelFunction','matern52','Standardize',...
        1);
    
    save(strcat(destinationFolder,'gprMdl_4_P.mat'),'model')
    
    
    
    %%
    X = tmp(:,[1:7 8]);
    y = tmp(:,9);
    rng default      
    model = fitrgp(X,y,'Sigma',0.08,'BasisFunction','constant',...
        'KernelFunction','matern52','Standardize',...
        0);
    
    
    save(strcat(destinationFolder,'gprMdl_4_PCG.mat'),'model')
    
    
    
    
    %%
    X = tmp(:,[1:6 8]);
    y = tmp(:,9);
    rng default    


    model = fitrgp(X,y,'Sigma',0.15888,'BasisFunction','constant',...
        'KernelFunction','matern52','Standardize',...
        1);
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
    %model= fitrgp(X,y,'Sigma',0.14131,'BasisFunction','pureQuadratic',...
    %    'KernelFunction','ardrationalquadratic','Standardize',...
    %    0);
    model= fitrgp(X,y,'BasisFunction','pureQuadratic',...
        'KernelFunction','ardrationalquadratic','Standardize',...
        0);
    save(strcat(destinationFolder,'gprMdl_20_P.mat'),'model')
    
    %%
    X = tmp(:,[1:6 8]);
    y = tmp(:,9);
    rng default    


    model = fitrgp(X,y,'Sigma',0.00011392,'BasisFunction','pureQuadratic',...
        'KernelFunction','rationalquadratic','Standardize',...
        1);
    
    save(strcat(destinationFolder,'gprMdl_20_PC.mat'),'model')
    %%
    X = tmp(:,[1:7 8]);
    y = tmp(:,9);
    rng default    


    model = fitrgp(X,y,'Sigma',0.013307,'BasisFunction','pureQuadratic',...
        'KernelFunction','rationalquadratic','Standardize',...
        1);
    
    save(strcat(destinationFolder,'gprMdl_20_PCG.mat'),'model')

    
   toc
    
end