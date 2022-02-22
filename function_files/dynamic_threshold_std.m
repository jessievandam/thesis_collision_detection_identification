function [dynamicThreshSTD, dynamicThreshSTDlpf] ...
                = dynamic_threshold_std(endInd,force, c1, Fstd, windowSize,...
                staticThresh, fc, timeVec)

%   Dynamic threshold based on standard deviation over fixed size time window

    % initialize
    dynamicThreshSTD = cell(2,1);
    dynamicThreshSTDlpf = cell(2,1);
    for i = 1:2
        dynamicThreshSTD{i,1} = zeros(3,endInd);
        dynamicThreshSTDlpf{i,1} = zeros(3,endInd);
    end
    
    % max standard deviation
    STDmax = [22.3234; 24.7723; 17.0872];
    
    for j = 1:2
        for i = 2:endInd
            if i<windowSize
                dynamicThreshSTD{j,1}(:,i) = min(c1.*std(force{j,1}(:,1:i),0,2)./STDmax,Fstd)...
                                        + staticThresh(:,i);         
            else
                 dynamicThreshSTD{j,1}(:,i) = min(c1.*std(force{j,1}(:,(i-windowSize+1):i),0,2)./STDmax,...
                                        Fstd) + staticThresh(:,i);                   
            end
        end
    end
    
    % adding LPF to the thresholds
    sysThresh = tf([0.0, fc*2*pi], [1.0, fc*2*pi]);
    dynamicThreshSTDlpf = cell(2,1);
    for j = 1:2
        for i = 1:3
            dynamicThreshSTDlpf{j,1}(i,:) = lsim(sysThresh,dynamicThreshSTD{j,1}(i,:),timeVec);
        end
    end
end

