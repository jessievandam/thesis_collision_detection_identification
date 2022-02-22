function [dynamicThresh, dynamicThreshLPF] = dynamic_threshold_velocity...
                    (staticThresh, K, qd_des, endInd, fc, timeVec)

%   Dynamic threshold based on velocity

    % max velocity per joint
%     load('qd_max');
%     qdMax = max(qd')'; 
%     qdMax = [0.0453; 0.0762; 0.0042; 0.0815; 0.0489; 0.0934; 0.0661; 0.0879; 
%         0.0998; 0.0669; 0.0794; 0.1504; 0.0653; 0.0757; 0.1132; 0.0636; 0.1045;
%         0.0808; 0.5739; 0.3466; 0.3267; 0.1152; 0.1046; 0.0590]; % armCtrlJoint_push_move
%     qdMax = [0.0730; 0.0671; 0.0661; 0.1942; 0.0577; 0.0375; 0.1149; 0.0795; 
%         0.4500; 0.1162; 0.0915; 0.4406; 0.1316; 0.2473; 0.1930; 0.1287; 0.2366; 
%         0.1842; 0.3638; 0.4961; 0.5317; 0.4739;  0.3093; 0.1539]; % arm_push_moving
%     qdMax = [0.0777; 0.0707; 0.0064; 0.1688; 0.0566; 0.0802; 0.1388; 0.1301;
%         0.1856; 0.1442; 0.1492; 0.1210; 0.0730; 0.1815; 0.1109; 0.2054; 0.1303;
%         0.1597; 0.6489; 0.6165; 0.6577; 0.5199; 0.1971; 0.4312]; % armctrl_armload_push
%     qdMax = [0.0753; 0.1188; 0.0078; 0.1331; 0.0386; 0.0832; 0.1284; 0.1050; 
%         0.1420; 0.1280; 0.1410; 0.1974; 0.0796; 0.1663; 0.1616; 0.0826; 0.0890; 
%         0.0951; 0.4159; 0.3449; 0.3176; 0.1608; 0.2385; 0.6957]; % armCtrlJoint_push
    qdMax = [0.0777; 0.0762; 0.0661; 0.1942; 0.0577; 0.0934; 0.1388; 0.1301; 
        0.4500; 0.1442; 0.1492; 0.4406; 0.1316; 0.2473; 0.1930; 0.2054; 0.2366; 
        0.1842; 0.6489; 0.6165; 0.6577; 0.5199; 0.3093; 0.6957]; % final
    
    % taking one norm and normalizing by qdmax  
    dynamicThresh = zeros(3,endInd);   
    for i = 1:endInd
        for j = 1:3     
            dynamicThresh(j,i) = staticThresh(j,1) + K(j,:) * (abs(qd_des(:,i))./qdMax(7:24));
        end                                          
    end
    % also try with max of two values just like with std threshold
   
    % adding LPF to the thresholds
    sysThresh = tf([0.0, fc*2*pi], [1.0, fc*2*pi]);
    dynamicThreshLPF = zeros(3, endInd);
    for i = 1:3
        dynamicThreshLPF(i,:) = lsim(sysThresh,dynamicThresh(i,:),timeVec);
    end
end

