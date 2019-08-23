clc;clear;

L = 0.01;
Fs = [];
K = zeros(2,2);
pts=[0,L/2,L];
for i = 1:1:3
    for j = 1:1:3
        K(i,j) = invKernel(pts(i),pts(j));
    end
end
K
det(K)
nominalD = 1*ones(3,1); %5mm all across..
actualD = K\nominalD;
totalF = 0;
for i=1:1:3
   totalF = totalF + linearForce(actualD(i)); 
end
totalF


