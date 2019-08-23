clc;clear;
load('originalData.mat');

ptCloud = pointCloud(pcd(:,1:3));
probedPoints(31,:) = [];
max = 0;
maxColor = 0;
for i=1:1:133
   pt = probedPoints(i,:);
   [indices,dists] = findNearestNeighbors(ptCloud,pt(1:3),1);
   color = pcd(indices,4:6);
   pt(1:3)
   disp(pcd(indices,1:3))
   if dists>max
       max = dists;
       %pt(1:3)
       %disp(pcd(indices,1:3))
   end
   if norm(color - pt(4:6)) > maxColor
       maxColor =  norm(color - pt(4:6));
       %color
       %pt(4:6)
   end
end
max
maxColor
   