function [dynamicThresh, dynamicThreshLPF] = dynamic_threshold_velocity...
                    (staticThresh, K, qd_des, endInd, fc, timeVec)

%   Dynamic threshold based on velocity

    % max velocity per joint
    qdMax = [0.0777; 0.0762; 0.0661; 0.1942; 0.0577; 0.0934; 0.1388; 0.1301; 
        0.4500; 0.1442; 0.1492; 0.4406; 0.1316; 0.2473; 0.1930; 0.2054; 0.2366; 
        0.1842; 0.6489; 0.6165; 0.6577; 0.5199; 0.3093; 0.6957]; 
    
    % taking one norm and normalizing by qdmax  
    dynamicThresh = zeros(3,endInd);   
    for i = 1:endInd
        for j = 1:3     
            dynamicThresh(j,i) = staticThresh(j,1) + K(j,:) * (abs(qd_des(:,i))./qdMax(7:24));
        end                                          
    end
    
    % adding LPF to the thresholds
    sysThresh = tf([0.0, fc*2*pi], [1.0, fc*2*pi]);
    dynamicThreshLPF = zeros(3, endInd);
    for i = 1:3
        dynamicThreshLPF(i,:) = lsim(sysThresh,dynamicThresh(i,:),timeVec);
    end
end

