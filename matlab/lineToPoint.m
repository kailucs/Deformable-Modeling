clc;clear;
%The linear probing data is a struct
%NofPoints:integer specifying the total numebr of collected pts
%field: -lineStarts is a M*3 matrix, where each row is the position of the
%line starting point
%-lineEnds is a M*3 matrix
%-lineNormals is a M*3 matrix
%-lineTorqueAxes is a M*3 matrix
%-data is a M*8? matrix, similar to that in the point case
%[color, k,d,f,tau]
linearData;

%% Model parameters
%Theta is a row vector of model parameters
%For each point, use 4th order polynomial      
%Use nonlinear regression with l2 regularization to learn the coefficients
%f = C1*d + C2*d^2 + C3*d^3 + C4*d^4 
%Need to train 4 separate models
weight = 10; %regularization weight
x0 = []; %initial guess for the features
%The C's need to be bounded.. but can't easily be written as constraints...
%and df/dd>=0


options = optimoptions('fmincon','Algorithm','interior-point','Display','none');
[x,fval]=fmincon(@(x)probingError(x,linearData,weight,pcd),x0,[],[],[],[],lb,ub,[],options);






%% 