clc;
clear;
DATAFOLDER = 'data/exp_1/';
DATATYPE = 0; %originial data
%DATATYPE = 1; %corrected data

tic;
if DATATYPE == 0
    load(strcat(DATAFOLDER,'originalData.mat'));
elseif DATATYPE ==1
    load('correctedData.mat');
end
t = toc;
disp(strcat('Data Loaded in',num2str(t),'s'))
%% Plotting

%%%%% plot collected data....

%% Top View of the testing object
figure
xRange =[-0.02 0.2];
yRange =[-0.07 0.18];
%pcd = diluteData(pcd,5);
pcd = diluteData(pcd,5);
for i = 1:1:size(pcd)
    %%pcd is the raw data, not scaled for learning
    x = pcd(i,1)-bias(1);
    y = pcd(i,2)-bias(2);
    z = pcd(i,3)-bias(3);
    color = pcd(i,4:6);
    plot(x,y,'.','Color',color,'MarkerSize',10)
    hold on
end

probedPoints(1,:)
%for i=1:1:size(probedPoints,1)
for i=[1]
        x = probedPoints(i,1)-bias(1);
        y = probedPoints(i,2)-bias(2);
        z = probedPoints(i,3)-bias(3);
        color = probedPoints(i,4:6);
        %h1 = plot(x,y,'.','Color',color,'MarkerSize',10);
        h1 = plot(x,y,'.g','MarkerSize',20);
end
% 
% h2 = plot(probedPoints(1,1)-bias(1),probedPoints(1,2)-bias(2),'.r','MarkerSize',14);
% h3 = plot(probedPoints(4,1)-bias(1),probedPoints(4,2)-bias(2),'.b','MarkerSize',14);
% h4 = plot(probedPoints(32,1)-bias(1),probedPoints(32,2)-bias(2),'.y','MarkerSize',14);
% h5 = plot(probedPoints(123,1)-bias(1),probedPoints(123,2)-bias(2),'.m','MarkerSize',14);
% 
% 
% legend([h1,h2,h3,h4,h5],{'Probed Points','Point 0','Point 3','Point 31','Point 122'})
axis equal
axis([xRange yRange])
xlabel('x (m)')
ylabel('y (m)')
title('Top View of the Testing Object')

%% Curvature of the testing object
% queryDisplacement = 0.005;
% xRange =[-0.02 0.1];
% yRange =[-0.01 0.16];
% colorBias = 0;
% colorScale = 1.0;
% colormap(hsv)
% for i = 1:1:size(pcd,1)
%     x = pcd(i,1)-bias(1);
%     y = pcd(i,2)-bias(2);
%     curvature = pcd(i,10);
%     curvature = (curvature-colorBias)/colorScale;
%     plot(x,y,'.','Color',hsv2rgb([curvature,1,1]),'MarkerSize',5)
%     hold on
% end
% axis equal
% axis([xRange yRange])
% xlabel('x (m)')
% ylabel('y (m)')
% title('Estimated Curvature Across the Testing Object ')
% hc = colorbar;
% set(hc,'TickLabels',{'0','0.1','0.2','0.3','0.4','0.5',...
%     '0.6','0.7','0.8','0.9','1.0'});


%% View of the Training and testing points
% figure
% xRange =[-0.02 0.1];
% yRange =[-0.01 0.16];
% trainingPtList = training4; 
% %pcd = diluteData(pcd,5);
% for i = 1:1:size(pcd)
%     %%pcd is the raw data, not scaled for learning
%     x = pcd(i,1)-bias(1);
%     y = pcd(i,2)-bias(2);
%     color = pcd(i,4:6);
%     plot(x,y,'.','Color',color,'MarkerSize',10)
%     hold on
% end
% 
% for i=1:1:size(trainingPtList,2)
%     index = trainingPtList(i);
%     x = probedPoints(index,1)-bias(1);
%     y = probedPoints(index,2)-bias(2);
%     h2 = plot(x,y,'g.','MarkerSize',13);
% end
% 
% legend([h2],{'Training Locations'})
% axis equal
% axis([xRange yRange])
% xlabel('x (m)')
% ylabel('y (m)')
% title('4 Training Locations')

%% 2D Ground Truth Force map (GIF)
% dispList = -8:1:5;
% dispList = dispList/1000;
% GIFDelay = 1;
% xRange =[-0.02 0.1];
% yRange =[-0.01 0.16];
% colorBias = -1.0;
% colorScale = 3.0;
% 
% max = -10;
% min = 10;
% im = {};
% frameCounter = 1;
% fig = figure;
% colormap(hsv)
% for iter = 1:1:size(dispList,2)
%     queryDisplacement = dispList(iter);                    
% 
%     for i = 1:1:size(probedPoints)
%         x = probedPoints(i,1)-bias(1);
%         y = probedPoints(i,2)-bias(2);
%         if DATATYPE == 0
%             groundTruthForce = queryForce(i,queryDisplacement);
%         elseif DATATYPE == 1
%             groundTruthForce = queryForceCorrected(i,queryDisplacement);
%         end
%         %%%%
%         if groundTruthForce > max
%             max = groundTruthForce;
%         end
%         if groundTruthForce < min
%             min = groundTruthForce;
%         end
%         %%%%
%         groundTruthForce = (groundTruthForce-colorBias)/colorScale;
%         if groundTruthForce > 1
%             force = 1;
%         elseif groundTruthForce < 0
%             force = 0;
%         end      
%         plot(x,y,'.','Color',hsv2rgb([groundTruthForce,1,1]),'MarkerSize',10)
%         hold on
%     end
%     axis equal
%     axis([xRange yRange])
%     xlabel('x (m)')
%     ylabel('y (m)')
%     title(strcat('Ground Truth Force at',{' '},num2str(queryDisplacement*1000),'mm'))
% 
%     hc = colorbar;
%     set(hc,'TickLabels',{'-1.0 N','-0.7 N','-0.4 N','-0.1 N','0.2 N','0.5 N',...
%         '0.8 N','1.1 N','1.4 N','1.7 N','2.0 N'})
% %     max
% %     min
% 
%     frame = getframe(fig);
%     im{frameCounter} = frame2im(frame);
%     frameCounter = frameCounter + 1;
% end
% 
% filename = strcat('pictures/ground_truth.gif');
% for idx = 1:1:frameCounter-1
%     [A,map] = rgb2ind(im{idx},256);
%     if idx == 1
%         imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',GIFDelay);
%     else
%         imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',GIFDelay);
%     end                
% end