%clc;clear;

L = 30; %30mm..
discretization = [1000];
Fs = [];
for n = 1:1:size(discretization,2)
    K = zeros(discretization(n),discretization(n));
    pts = linspace(0,30,discretization(n));
    for i = 1:1:discretization(n) 
        for j = 1:1:discretization(n)
            K(i,j) = linearKernel(pts(i),pts(j));
        end
    end
    det(K)
    nominalD = 5*ones(discretization(n),1); %5mm all across..
    %nominalD = linspace(5,7,discretization(n))';
    actualD = K\nominalD;
    totalF = 0;
    for i=1:1:discretization(n)
       totalF = totalF + linearForce(actualD(i)); 
    end
    Fs=[Fs;totalF];
    actualD
end

%% Plot how these "kernels" behave
% x = 0:0.1:30;
% Y=[];
% for i=1:1:size(x,2)
%     Y=[Y [linearKernel(0,x(i));invKernel(0,x(i));inv2Kernel(0,x(i))]];
% end
% 
% plot(x,Y(1,:),'r',x,Y(2,:),'b',x,Y(3,:),'g')
% legend('linear','inverse','inverse squared')

%%plot a single push with different "kernels"
x=-10:0.1:10;
Y=[];
d=5;
for i=1:1:size(x,2)
   Y = [Y -5*[linearKernel(0,x(i));invKernel(0,x(i));inv2Kernel(0,x(i))]]; 
end
plot(x,Y(1,:),'r',x,Y(2,:),'b',x,Y(3,:),'g')
legend('linear','inverse','inverse squared')
axis equal