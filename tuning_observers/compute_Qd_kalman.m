%% open velocities and time legs and arms

% velocities legs and arms
qdCell = cell(18,1);
timeCell = cell(18,1);

% open parameters arm calibration constant velocity 0.2
load('qdArm');
load('timeArm');
for i = 1:6
    qdCell{12+i,1} = qd(i,:);
    timeCell{12+i,1} = time;    
end

% open parameter legs
load('qdLegs');
load('timeLegs');
for i = 1:12
    qdCell{i,1} = qdTotal(i,:);
    timeCell{i,1} = timeTotal;    
end

%% plot velocities to determine time intervals with constant velocity

% joints to plot (1 to 18, excluding base)
j1 = 10;
j2 = 11;
j3 = 12;

% plot
plot_velocities(timeCell, qdCell, j1, j2, j3);

% save intervals with constant velocity, see next code block variable 'times'

%% compute velocity covariance matrix Qqd during arm movement

% initialize
times = zeros(18,4);
times(1,:) = [17.5;19.5; 30;32];
times(2,:) = [43;52; 56;64];
times(3,:) = [66;76; 78;88];
times(4,:) = [91;93; 103.5;105];
times(5,:) = [116;124; 128;136];
times(6,:) = [140;148; 152;161];
times(7,:) = [164.5;166; 176.5;178.5];
times(8,:) = [190;198; 202;210];
times(9,:) = [213;222; 225;234];
times(10,:) = [237.5;239.5; 250;251.5];
times(11,:) = [262;271; 276;284];
times(12,:) = [286;296; 298;308];
times(13,:) = [189.5;197; 204;211]; % extra: 217;225; 232;238
times(14,:) = [159;162; 168;170];   % extra: 175.5;177.5; 182.5;184
times(15,:) = [140;144; 150;154];   % extra: 126.8;128.5; 133.4;135
times(16,:) = [80;87; 93;100];      % extra: 106;111; 116;121
times(17,:) = [46.5;52; 57.5;61];
times(18,:) = [26.5;31;  37;41.5];
indices = zeros(18,4);
for k = 1:18
    for i = 1:4
        indices(k,i) = find(timeCell{k,1}(1,:)>times(k,i),1);
    end
end

% calculate mean over intervals with constant velocity
meanQd = zeros(18,2);
qdMeanSubtracted = cell(18,1);
for k = 1:18
    qdMeanSubtracted{k,1} = cell(2,1);
end
for k = 1:18
    for i = 1:2
        meanQd(k,i) = mean(qdCell{k,1}(1,  indices(k,i*2-1):indices(k,i*2)));
        qdMeanSubtracted{k,1}{i,1} = qdCell{k,1}(1, indices(k,i*2-1):indices(k,i*2)) - meanQd(k,i);
    end
end

% calculate standard deviation
S = zeros(6,2);
for k = 1:18
    for i = 1:2
        S(k,i) = std(qdMeanSubtracted{k,1}{i,1}); % standard deviation of qd - qdmean
    end
end
Sfinal = mean(S,2);

% save in diagonal matrix for kalman
Qqd = zeros(24,24);
Qqd(7:24,7:24) = diag(Sfinal);
% save('Qqd','Qqd')
% dlmwrite('Qqd.txt', Qqd, 'delimiter','\t','newline','pc')

