function sum = objectFunc3(C,forceData,scale)
sum = 0;
C(1) = C(1)/scale;
for i=1:1:size(forceData,1)
   if forceData(i,1)<C(1) %put less weight on the "zero points"
        sum = sum + 0.1*(forceData(i,2)-forceModel2(forceData(i,1),C))^2; 
   else
        sum = sum + (forceData(i,2)-forceModel2(forceData(i,1),C))^2; 
   end
end
%penalize x(2), the intercept from being nonzero
sum = sum+1*(C(2)^2+C(3)^2+C(4)^2+C(5)^2)+ C(1)^2;
end

