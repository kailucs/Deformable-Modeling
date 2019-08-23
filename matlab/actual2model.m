function data = actual2model(input,dataRange,columnMin)
%the 9th element is the force
%input here does not include force..
data = input;
dataRange = dataRange(1:8);
columnMin = columnMin(1:8);
data = data - repmat(columnMin,size(data,1),1);
%scale the data such that they are roughly on the same scale
data(:,1:3) = data(:,1:3)*(1/max(dataRange(1:3)));
data(:,4:6) = data(:,4:6)*(1/max(dataRange(4:6)));
data(:,7) = data(:,7)*(1/dataRange(7));
data(:,8) = data(:,8)*(1/dataRange(8));
end

