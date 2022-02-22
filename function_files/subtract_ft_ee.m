function [ftForceEELPF, forceMinusFTEE, magEstForceMinusFTEE, magEstForceLPFMinusFTEE] =...
                    subtract_ft_ee(bag, magEstForce, magEstForceLPF, force,...
                    endInd, timeVec, n3sec, n4sec, name)

%   Subtract forces FT sensor EE

    %% read EE wrench from bag
    
    ftEE_bag = select(bag, 'Topic',strcat(name,'/ft_sensor_wrench_hw'));
    ftEE_struct = readMessages(ftEE_bag,'DataFormat','struct');
    
    %% read FT sensor EE from struct
    
    if name == "/collision_detection"
        
        % measured force still shows offset
        ftForceMeasEE = zeros(3,endInd);
        for i = 1:endInd
            ftForceMeasEE(1,i) = ftEE_struct{i,1}.Force.X;
            ftForceMeasEE(2,i) = ftEE_struct{i,1}.Force.Y;
            ftForceMeasEE(3,i) = ftEE_struct{i,1}.Force.Z;
        end
        
        % time 
        timeftEE = ftEE_bag.MessageList.Time;
        timeftEE = timeftEE - timeftEE(1,1);
       
        % subtract offset
        n1 = find(timeftEE>n3sec,1);
        n2 = find(timeftEE>n4sec,1);
        offset(1,1) = mean(ftForceMeasEE(1,n1:n2));
        offset(2,1) = mean(ftForceMeasEE(2,n1:n2));
        offset(3,1) = mean(ftForceMeasEE(3,n1:n2));

        ftForceEE = zeros(3,endInd);
        for i = 1:endInd
            ftForceEE(1,i) = ftForceMeasEE(1,i)-offset(1,1);
            ftForceEE(2,i) = ftForceMeasEE(2,i)-offset(2,1);
            ftForceEE(3,i) = ftForceMeasEE(3,i)-offset(3,1);
        end
    else
        ftForceEE = zeros(3,endInd);
        for i = 1:endInd
            ftForceEE(1,i) = ftEE_struct{i,1}.Force.X;
            ftForceEE(2,i) = ftEE_struct{i,1}.Force.Y;
            ftForceEE(3,i) = ftEE_struct{i,1}.Force.Z;
        end
    end

    %% add LPF
    
    fc = 4.0; % [Hz]
    sysLPF = tf([0.0, fc*2*pi], [1.0, fc*2*pi]);
    ftForceEELPF = zeros(3,endInd);
    for i = 1:3
        ftForceEELPF(i,:) = lsim(sysLPF,ftForceEE(i,:),timeVec);
    end
    
    %% subtract EE force from estimated force
    % note: orientation in data set 24-01 is incorrect, so this force as well
    
    K = size(magEstForce,1);
    H = size(magEstForce,2);
    
    forceMinusFTEE = cell(K,H);
    for k = 1:K
        for h = 1:H
            forceMinusFTEE{k,h} = force{k,h} - ftForceEELPF;  
        end 
    end
    
    %% compute magnitude of EE force
    
    magEstForceFTEE = sqrt(ftForceEE(1,:).^2 + ftForceEE(2,:).^2 + ftForceEE(3,:).^2);
    magEstForceFTEELPF = sqrt(ftForceEELPF(1,:).^2 + ftForceEELPF(2,:).^2 + ftForceEELPF(3,:).^2);
    
    %% subtract EE force from magnitude estimated force
    
    magEstForceMinusFTEE = cell(K,H);
    magEstForceLPFMinusFTEE = cell(K,H);
    for k = 1:K
        for h = 1:H
            magEstForceMinusFTEE{k,h} = magEstForce{k,h}(1,:) - magEstForceFTEE(1,:);
            magEstForceLPFMinusFTEE{k,h} = magEstForceLPF{k,h}(1,:) - magEstForceFTEELPF(1,:);
        end
    end
end

