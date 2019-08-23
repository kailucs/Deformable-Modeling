function ypred = knnPredict(data,k,query)

ypred = [];
NofFeatures = size(data,2)-1;

%scale displacements
scale = 50;
data(:,NofFeatures) = data(:,NofFeatures)*scale;
query(:,NofFeatures) = query(:,NofFeatures)*scale;
    for i = 1:1:size(query,1)
        [Idx,~] = knnsearch(data(:,1:NofFeatures),query(i,:),'K',k);
        neighbors = data(Idx,NofFeatures+1);  
        ypred = [ypred;mean(neighbors)];
    end

end

