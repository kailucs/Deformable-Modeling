function force = queryForceCorrected(index,displacement)
    %The queried force would be filtered..
    
    forceData = dlmread(strcat('data_bias_removed/force',num2str(index-1),'.txt'),' ');
    forceData = forceData(:,[2 1]);
    Wn=0.025;
    [b,a]=butter(5,Wn,'low');
    filteredF=filtfilt(b,a,forceData(:,2));
    forceData(:,2) = filteredF;
    
    force = 0;
    for i=size(forceData,1):-1:1
        if displacement >= forceData(i,1)
            force = forceData(i,2);
            break
        end
    end
end

