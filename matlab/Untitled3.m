clc;clear;



pcd = dlmread(strcat('data/exp_4_debiased/','originalPcd.txt'),' ');
probedPoints = dlmread(strcat('data/exp_4_debiased/','probePcd.txt'),' ');
pcshowpair(pointCloud(diluteData(pcd(:,1:3),1)),pointCloud(probedPoints(:,1:3)),'MarkerSize',40);
   
% sourceFolder = 'data/';
% destinationFolder = 'data_bias_removed/';
% for i=31:1:60
%     plotForce(i)
% end

% 
%     for i=1:1:1
%         
%          forceData = dlmread(strcat(sourceFolder,'force',num2str(i-1),'.txt'),' ');
%          forceData = forceData(:,1:2);
%          fileName = strcat(destinationFolder,'force',num2str(i-1),'.txt');
%          fileID = fopen(fileName,'w');
%          forceData(:,2) = forceData(:,2)-repmat(0,size(forceData,1),1);
%          size(forceData)
%          %fprintf(fileID,'%12.5f %12.8f\n',forceData.');
%          fprintf(fileID,'%f %f\n',forceData.');
%          fclose(fileID);
%     end
%     
%     forceData = dlmread(strcat(destinationFolder,'force',num2str(i-1),'.txt'),' ');
%     size(forceData)