function data = modifyData(input,dataRange,columnMin)
%scale the data s.t. it is between 0 and 1
data = input;
columnMin(9) = 0;
data = data - repmat(columnMin,size(data,1),1);
%scale the data such that they are roughly on the same scale
data(:,1:3) = data(:,1:3)*(1/max(dataRange(1:3)));
data(:,4:6) = data(:,4:6)*(1/max(dataRange(4:6)));
data(:,7) = data(:,7)*(1/dataRange(7));
data(:,8) = data(:,8)*(1/dataRange(8));
end

