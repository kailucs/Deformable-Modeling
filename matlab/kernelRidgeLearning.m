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

%%All the training are already biased and scaled..
destinationFolder = 'trained_model_3/';
diluteFactor = 8; 
tmp = diluteData(trainingData20,diluteFactor);
X = tmp(:,[1:3 8]);
y = tmp(:,9);
lb=[];ub=[];
%Let's use position only
%use the kernel to extend this to higher dimension
%from 3 D to 9D
x0 = zeros(1,9*5);
weight = 10;
options = optimoptions('fmincon','Algorithm','interior-point','Display','iter',...
    'UseParallel',true);
[x,fval]=fmincon(@(x)ptProbingError(x,tmp,weight),x0,[],[],[],[],lb,ub,[],options);

model = x;
save('trained_model_3/krMdl_20_P.mat','model')