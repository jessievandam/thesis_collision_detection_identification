function [] = plot_filtering_freq_domain(magFTForce,timeftShifted,forceFiltered1, forceFiltered2,...
                         forceFiltered3, forceFiltered4, forceName1, forceName2, forceName3, forceName4,...
                         bodyPart, time, startTime, endTime)

%   Plot the raw and filtered forces in freq domain to tune

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
    plot(time(n:m1),  forceFiltered1(1,n:m1), 'r', 'LineWidth', LW)
    hold on
    plot(time(n:m1),  forceFiltered2(1,n:m1), 'b', 'LineWidth', LW)
    if ~isempty(forceFiltered3)
        hold on
        plot(time(n:m1),  forceFiltered3(1,n:m1), 'k', 'LineWidth', LW)
    end
    if ~isempty(forceFiltered4)
        hold on
        plot(time(n:m1),  forceFiltered4(1,n:m1), 'g', 'LineWidth', LW)
    end
    hold off
    grid on
    title(bodyPart + " force x magnitude",'Interpreter','latex','Fontsize', FS)
    ylabel('Power [dB]','Interpreter','latex','Fontsize', FS)
    xlim([time(nn) time(mm)])

    subplot(4,1,2)
    plot(time(n:m1),  forceFiltered1(2,n:m1), 'r', 'LineWidth', LW)
    hold on
    plot(time(n:m1),  forceFiltered2(2,n:m1), 'b', 'LineWidth', LW)
    if ~isempty(forceFiltered3)
        hold on
        plot(time(n:m1),  forceFiltered3(2,n:m1), 'k', 'LineWidth', LW)
    end
    if ~isempty(forceFiltered4)
        hold on
        plot(time(n:m1),  forceFiltered4(2,n:m1), 'g', 'LineWidth', LW)
    end
    hold off
    grid on
    title(bodyPart + " force y magnitude",'Interpreter','latex','Fontsize', FS)
    ylabel('Power [dB]','Interpreter','latex','Fontsize', FS)
    leg = legend(forceName1,forceName2, forceName3, forceName4);
    set(leg, 'Location', 'northeast',  'Interpreter', 'latex','Fontsize', 12);
    xlim([time(nn) time(mm)])

    subplot(4,1,3)
    plot(time(n:m1),  forceFiltered1(3,n:m1), 'r', 'LineWidth', LW)
    hold on
    plot(time(n:m1),  forceFiltered2(3,n:m1), 'b', 'LineWidth', LW)
    if ~isempty(forceFiltered3)
        hold on
        plot(time(n:m1),  forceFiltered3(3,n:m1), 'k', 'LineWidth', LW)
    end
    if ~isempty(forceFiltered4)
        hold on
        plot(time(n:m1),  forceFiltered4(3,n:m1), 'g', 'LineWidth', LW)
    end
    hold off
    grid on
    title(bodyPart + " force z magnitude",'Interpreter','latex','Fontsize', FS)
    ylabel('Power [dB]','Interpreter','latex','Fontsize', FS)
    xlim([time(nn) time(mm)])

    subplot(4,1,4)
    plot(timeftShifted(n:m2), magFTForce(n:m2), 'k', 'LineWidth', LW)
    hold off
    grid on
    title('FT sensor force collision','Interpreter','latex','Fontsize', FS)
    ylabel('Force[N]','Interpreter','latex','Fontsize', FS)
    xlim([time(nn) time(mm)])
    xlabel('Time [sec]','Interpreter','latex','Fontsize', FS)
    
%     exportgraphics(gcf, 'fig_comparison_withandwithouthann.png', 'Resolution', 300);
end

