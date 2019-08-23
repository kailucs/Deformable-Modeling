function ypred = kernelRidgePredict(model,query)
%model is a list of weights for input features
%column vector
totalNofWeights = size(model,2);
model0 = model(1:totalNofWeights/5);%weights for the first degree of the polynomial
model1 = model(totalNofWeights/5+1:2*totalNofWeights/5);
model2 = model(2*totalNofWeights/5+1:3*totalNofWeights/5);
model3 = model(3*totalNofWeights/5+1:4*totalNofWeights/5);
model4 = model(4*totalNofWeights/5+1:totalNofWeights);
%each row of query is one query, containing the deatures
%the last th
ypred = [];
for i=1:1:size(query,1)
   d = query(i,end);
   O = dot(model0,learningKernelFunc(query(i,1:end-1)));
   A = dot(model1,learningKernelFunc(query(i,1:end-1)));
   B = dot(model2,learningKernelFunc(query(i,1:end-1)));
   C = dot(model3,learningKernelFunc(query(i,1:end-1)));
   D = dot(model4,learningKernelFunc(query(i,1:end-1)));
   f = O + A*d + B*d^2 + C*d^3 + D*d^4;
   ypred = [ypred;f];
end
end

