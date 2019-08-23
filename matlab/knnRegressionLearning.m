clc;
clear;
%%%%%
MODE = 0;%learn
DATATYPE = 1;
%%%%%
DATAFOLDER1 = 'data/exp_1/';
DATAFOLDER2 = 'data/exp_1_debiased/';
modelFolder = 'data/exp_1_debiased/trained_model/';
if DATATYPE == 0
    load(strcat(DATAFOLDER1,'originalData.mat'));
elseif DATATYPE ==1
    load(strcat(DATAFOLDER2,'debiasedData.mat'));
end

diluteFactor = 40; %% this has to be chosen such that we don't have too many
%force displacement-force pairs.....
%~25 points along displacement-force
%%
if MODE == 0
    tic
    % [position(3) color(3) curvature(1) displacement(1) force(1)]
    %%20 points
    tmp = diluteData(trainingData,diluteFactor);
    size(tmp)
    X = tmp(:,[1:3 8]);
    y = tmp(:,9);
    model = [X y];
    save(strcat(modelFolder,'knnMdl_121_P.mat'),'model')
    
    %%
    X = tmp(:,[1:6 8]);
    y = tmp(:,9);
    model = [X y];
    save(strcat(modelFolder,'knnMdl_121_PC.mat'),'model')
    %%
    X = tmp(:,[1:7 8]);
    y = tmp(:,9);
    model = [X y];
    save(strcat(modelFolder,'knnMdl_121_PCG.mat'),'model')
   toc
end