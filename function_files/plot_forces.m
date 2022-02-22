function [] = plot_forces(magFTForce,magFTForceEE,magFTForceEElpf, timeftShifted,...
                         forceHPF, bodyPart, time, startTime, endTime)
                     
%   Plot the forces in all directions

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
    plot(time(n:m1),  (forceHPF(1,n:m1)), 'r', 'LineWidth', LW)
    hold off
    grid on
    title(bodyPart + " force x",'Interpreter','latex','Fontsize', FS)
    ylabel('Force [N]','Interpreter','latex','Fontsize', FS)
    xlim([time(nn) time(mm)])
    
    subplot(4,1,2)
    plot(time(n:m1),  (forceHPF(2,n:m1)), 'r', 'LineWidth', LW)
    hold off
    grid on
    title(bodyPart + " force y",'Interpreter','latex','Fontsize', FS)
    ylabel('Force [N]','Interpreter','latex','Fontsize', FS)
    xlim([time(nn) time(mm)])

    subplot(4,1,3)
    plot(time(n:m1), (forceHPF(3,n:m1)), 'r', 'LineWidth', LW)
    hold off
    grid on
    title(bodyPart + " force z",'Interpreter','latex','Fontsize', FS)
    ylabel('Force [N]','Interpreter','latex','Fontsize', FS)
    xlim([time(nn) time(mm)])

    subplot(4,1,4)
    plot(timeftShifted(n:m2), magFTForce(n:m2), 'k', 'LineWidth', LW)
    if ~isempty(magFTForceEE)
        hold on
        plot(time(n:m1),  magFTForceEE(n:m1), 'r', 'LineWidth', LW)
    end
    if ~isempty(magFTForceEElpf)
        hold on
        plot(time(n:m1),  magFTForceEElpf(n:m1), 'b', 'LineWidth', LW)
    end
    hold off
    grid on
    title('FT sensor force magnitude','Interpreter','latex','Fontsize', FS)
    ylabel('Force[N]','Interpreter','latex','Fontsize', FS)
    xlim([time(nn) time(mm)])
    xlabel('Time [sec]','Interpreter','latex','Fontsize', FS)
    leg = legend('collision force','EE force', 'EE force+LPF');
    set(leg, 'Location', 'northeast',  'Interpreter', 'latex','Fontsize', 15);

end

