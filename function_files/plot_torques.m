function [] = plot_torques(time, timeftShifted, torques, magFTForce, thresh, j1, j2, j3, nnsec, mmsec)

%   Plot joint torques

    % start and end index
    n = find(time>0,1);
    m1 = find(time>(min(time(end), timeftShifted(end))-1),1);          % estimated force endInd
    m2 = find(timeftShifted>(min(time(end), timeftShifted(end))-1),1); % ft sensor collision endInd

    % interval to plot
    nn = find(time>nnsec,1);
    mm = find(time>mmsec,1);
    
    % figure params
    LW = 1;
    FS = 20;
    x00=10;
    y00=10;
    width=1000;
    height=1400;

    figure()
    set(gcf,'position',[x00,y00,width,height/2])
    subplot(4,1,1)
    plot(time(n:m1),  torques(j1,(n:m1)), 'b', 'LineWidth', LW)
    if ~isempty(thresh)
        hold on
        plot(time(n:m1),  (thresh(1,n:m1)), 'k--', 'LineWidth', 2)
    end
    hold off
    grid on
    title("Torque arm joint " + (j1-18),'Interpreter','latex','Fontsize', FS)
    ylabel('Torque [Nm]','Interpreter','latex','Fontsize', FS)
    xlim([time(nn) time(mm)])

    subplot(4,1,2)
    plot(time(n:m1),  torques(j2,(n:m1)), 'b', 'LineWidth', LW)
    if ~isempty(thresh)
        hold on
        plot(time(n:m1),  (thresh(2,n:m1)), 'k--', 'LineWidth', 2)
    end
    hold off
    grid on
    title("Torque arm joint " + (j2-18),'Interpreter','latex','Fontsize', FS)
    ylabel('Torque [Nm]','Interpreter','latex','Fontsize', FS)
    xlim([time(nn) time(mm)])

    subplot(4,1,3)
    plot(time(n:m1),  torques(j3,(n:m1)), 'b', 'LineWidth', LW)
    if ~isempty(thresh)
        hold on
        plot(time(n:m1),  (thresh(3,n:m1)),'k--', 'LineWidth', 2)
    end
    hold off
    grid on
    title("Torque arm joint " + (j3-18),'Interpreter','latex','Fontsize', FS)
    ylabel('Torque [Nm]','Interpreter','latex','Fontsize', FS)
    xlim([time(nn) time(mm)])
    
    subplot(4,1,4)
    plot(timeftShifted(n:m2),  magFTForce(n:m2), 'b', 'LineWidth', LW)
    hold off
    grid on
    title("Magnitude FT sensor ground truth force",'Interpreter','latex','Fontsize', FS)
    ylabel('Force [N]','Interpreter','latex','Fontsize', FS)
    xlabel('Time [sec]','Interpreter','latex','Fontsize', FS)
    xlim([time(nn) time(mm)])

end