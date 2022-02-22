function [] = plot_velocities(timeCell, qdCell, j1, j2, j3)

%   Plot joint velocities

    % start and end index
    n = 1;
    m = size(timeCell{j1,1},2);

    % figure params
    LW = 1;
    FS = 20;
    x00=10;
    y00=10;
    width=1000;
    height=1400;

    figure()
    set(gcf,'position',[x00,y00,width,height/2])
    subplot(3,1,1)
    plot(timeCell{j1,1}(n:m), qdCell{j1,1}(n:m), 'r', 'LineWidth', LW)
    hold off
    grid on
    title("Velocity joint " + j1, 'Interpreter','latex','Fontsize', FS)
    ylabel('Velocity [rad/sec]','Interpreter','latex','Fontsize', FS)
    xlim([timeCell{j1,1}(n) timeCell{j1,1}(m)])

    subplot(3,1,2)
    plot(timeCell{j2,1}(n:m), qdCell{j2,1}(n:m), 'r', 'LineWidth', LW)
    hold off
    grid on
    title("Velocity joint " + j2, 'Interpreter','latex','Fontsize', FS)
    ylabel('Velocity [rad/sec]','Interpreter','latex','Fontsize', FS)
    xlim([timeCell{j1,1}(n) timeCell{j1,1}(m)])

    subplot(3,1,3)
    plot(timeCell{j3,1}(n:m), qdCell{j3,1}(n:m), 'r', 'LineWidth', LW)
    hold off
    grid on
    title("Velocity joint " + j3, 'Interpreter','latex','Fontsize', FS)
    ylabel('Velocity [rad/sec]','Interpreter','latex','Fontsize', FS)
    ylabel('Time [sec]','Interpreter','latex','Fontsize', FS)
    xlim([timeCell{j1,1}(n) timeCell{j1,1}(m)])

end

