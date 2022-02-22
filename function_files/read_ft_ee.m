function [ftForceEE, ftForceEELPF, magFTForceEE, magFTForceEElpf,...
            forceMinusEE] = read_ft_ee(bag, endInd,time, timeVec, force)

%   Read parameters FT sensor EE


    %% read out FT sensor values collision

    % FT EE calibrated wrench in world frame
    ftEE_bag = select(bag, 'Topic','/ocs2_controller_collision/ft_sensor_wrench_hw');
    ftEE_struct = readMessages(ftEE_bag,'DataFormat','struct');
    
    % read values
    ftForceEE = zeros(3,endInd);
    for i = 1:endInd
        ftForceEE(1,i) = ftEE_struct{i,1}.Force.X;
        ftForceEE(2,i) = ftEE_struct{i,1}.Force.Y;
        ftForceEE(3,i) = ftEE_struct{i,1}.Force.Z;
    end

    %% apply LPF to force

    fc = 4.0; % [Hz]
    sysLPF = tf([0.0, fc*2*pi], [1.0, fc*2*pi]);
    ftForceEELPF = zeros(3,endInd);
    for i = 1:3
        ftForceEELPF(i,:) = lsim(sysLPF,ftForceEE(i,:),timeVec);
    end

    %% compute magnitude 
    
    magFTForceEE = sqrt(ftForceEE(1,:).^2 + ftForceEE(2,:).^2 + ftForceEE(3,:).^2);
    magFTForceEElpf = sqrt(ftForceEELPF(1,:).^2 + ftForceEELPF(2,:).^2 + ftForceEELPF(3,:).^2);
    
    %% subtract EE force from current force estimate
    forceMinusEE = cell(2,1);
    for h = 1:2 % loop over base and arm jacobian
        for i = 1:endInd  
            forceMinusEE{h,1}(:,i) = force{h,1}(:,i)-ftForceEE(:,i);
        end
    end
   
end
