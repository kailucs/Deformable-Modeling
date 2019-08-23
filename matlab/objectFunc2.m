function sum = objectFunc2(C,forceData,scale)
sum = 0;
C(1) = C(1)/scale;
for i=1:1:size(forceData,1)
    sum = sum + (forceData(i,2)-forceModel(forceData(i,1),C))^2; 
end
sum = sum+0.1*(C(3)^2+C(4)^2+C(5)^2+C(6)^2)+100*C(2)^2;
end

