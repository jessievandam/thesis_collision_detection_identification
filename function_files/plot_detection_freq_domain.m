function [] = plot_detection_freq_domain(magFTForce,timeftShifted,forceFiltered, time,...
                            startTime, endTime, constThresh, collision)

%   Plot forces filtered in frequency domain with constant and dynamic threshold

    % parameters figure
    LW = 1;
    FS = 20;
    x00=10;
    y00=10;
    width=1000;
    height=1400;

    % time to be plotted
    nn = find(time>startTime,1); 
    mm = find(time>endTime,1);  

    % synchronize time interval (since different time steps are used) for estimated and ground truth force
    n = find(time>0,1);
    m1 = find(time>(min([time(end), timeftShifted(end)])-1),1);          % estimated force endInd
    m2 = find(timeftShifted>(min([time(end), timeftShifted(end)])-1),1); % ft sensor collision endInd

    figure()
    set(gcf,'position',[x00,y00,width,height/2])

    subplot(4,1,1)
    plot(time(n:m1),  forceFiltered(1,n:m1), 'r', 'LineWidth', LW)
    hold on
    plot(time(n:m1),  constThresh(1,n:m1), 'k', 'LineWidth', 2)
    hold off
    grid on
    title('Arm force x magnitude','Interpreter','latex','Fontsize', FS)
    ylabel('Power [dB]','Interpreter','latex','Fontsize', FS)
    xlim([time(nn) time(mm)])

    subplot(4,1,2)
    plot(time(n:m1),  forceFiltered(2,n:m1), 'r', 'LineWidth', LW)
    hold on
    plot(time(n:m1),  constThresh(2,n:m1), 'k', 'LineWidth', 2)
    hold off
    grid on
    title('Arm force y magnitude','Interpreter','latex','Fontsize', FS)
    ylabel('Power [dB]','Interpreter','latex','Fontsize', FS)
    leg = legend('filtered force','const thresh');
    set(leg, 'Location', 'northeast',  'Interpreter', 'latex','Fontsize', 12);
    xlim([time(nn) time(mm)])

    subplot(4,1,3)
    plot(time(n:m1),  forceFiltered(3,n:m1), 'r', 'LineWidth', LW)
    hold on
    plot(time(n:m1),  constThresh(3,n:m1), 'k', 'LineWidth', 2)
    hold off
    grid on
    title('Arm force z magnitude','Interpreter','latex','Fontsize', FS)
    ylabel('Power [dB]','Interpreter','latex','Fontsize', FS)
    xlim([time(nn) time(mm)])

    subplot(4,1,4)
    plot(timeftShifted(n:m2), magFTForce(n:m2), 'k', 'LineWidth', LW)
    hold on
    plot(time(n:m1), 10*collision(n:m1), 'r', 'LineWidth', LW)
    hold off
    grid on
    title('FT sensor force collision','Interpreter','latex','Fontsize', FS)
    ylabel('Force[N]','Interpreter','latex','Fontsize', FS)
    leg = legend('FT force','collision?');
    set(leg, 'Location', 'east',  'Interpreter', 'latex','Fontsize', 12);
    xlim([time(nn) time(mm)])
    xlabel('Time [sec]','Interpreter','latex','Fontsize', FS)
 
%     exportgraphics(gcf, 'fig_armfilteredfreq.png', 'Resolution', 300);
end

