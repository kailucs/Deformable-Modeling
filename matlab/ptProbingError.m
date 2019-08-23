function e = ptProbingError(model,trainingData,weight)

ypred = kernelRidgePredict(model,trainingData(:,[1:3 8]));
e = sum((ypred-trainingData(:,end)).^2) + weight*sum(model.^2);

end

