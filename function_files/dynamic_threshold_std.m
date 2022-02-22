function [dynamicThreshSTD, dynamicThreshSTDlpf, dynamicThreshSTD2, dynamicThreshSTDlpf2] ...
                = dynamic_threshold_std(endInd,force, c1, Fstd, windowSize,...
                staticThresh, fc, timeVec)

%   Dynamic threshold based on standard deviation over fixed size time window

    dynamicThreshSTD = zeros(3,endInd);
    dynamicThreshSTD2 = zeros(3,endInd);
%     STDmax = [15.5796; 20.3467; 14.4651];  % armCtrlJoint_push_move
%     STDmax = [11.1473; 14.0632; 17.0872];  % arm_push_moving
%     STDmax = [22.3234; 24.7723; 15.9875];  % armctrl_armload_push
%     STDmax = [17.3405; 18.0858; 14.7308]; % armCtrlJoint_push
    STDmax = [22.3234; 24.7723; 17.0872]; % final
    
    for i = 2:endInd
        if i<windowSize
            dynamicThreshSTD(:,i) = min(c1.*std(force(:,1:i),0,2)./STDmax,Fstd)...
                                    + staticThresh(:,i);
%            STD(:,i) = std(force(:,1:i),0,2);                     
        else
             dynamicThreshSTD(:,i) = min(c1.*std(force(:,(i-windowSize+1):i),0,2)./STDmax,...
                                    Fstd) + staticThresh(:,i);  
%            STD(:,i) = std(force(:,(i-windowSize+1):i),0,2);                     
        end
        
        % when varying tuning constants
        if i<windowSize
           dynamicThreshSTD2(:,i) = min(c1.*std(force(:,1:i),0,2)./STDmax,Fstd)...
                                    + staticThresh(:,i);
        else
           dynamicThreshSTD2(:,i) = min(c1.*std(force(:,(i-windowSize+1):i)./STDmax,0,2),...
                                    Fstd) + staticThresh(:,i);                    
        end
    end
    
    % adding LPF to the thresholds
    sysThresh = tf([0.0, fc*2*pi], [1.0, fc*2*pi]);
    dynamicThreshSTDlpf = zeros(3, endInd);
    dynamicThreshSTDlpf2 = zeros(3, endInd);
    for i = 1:3
        dynamicThreshSTDlpf(i,:) = lsim(sysThresh,dynamicThreshSTD(i,:),timeVec);
        dynamicThreshSTDlpf2(i,:) = lsim(sysThresh,dynamicThreshSTD2(i,:),timeVec);
    end
end

