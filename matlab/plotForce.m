function plotForce(index)
    figure
    tic
    forceData = dlmread(strcat('data/exp_4_debiased/point/force_',num2str(index-1),'.txt'),' ');
    %forceData = dlmread(strcat('experiment_data_steak_06102019/corrected_force/force',num2str(index-1),'.txt'),' ');
    forceData = forceData(:,[5 4]);
    Wn=0.025;
    [b,a]=butter(5,Wn,'low');
    filteredF=filtfilt(b,a,forceData(:,2));
    forceData(:,1) = forceData(:,1)*1000;
    %plot(forceData(:,1),forceData(:,2),'b')
    axis([-10 6 -0.5 2.5])
    xlabel('Poking Displacement (mm)')
    ylabel('F (N)')
    title(strcat('Force in a Single Probing Action at Point',{' '},num2str(index-1)))
    plot(forceData(:,1),forceData(:,2),'b',forceData(:,1),filteredF,'r')
    grid on
    toc
end
