function [time, endInd, timeVec, Ts, nonlinearTerms, q, qd, qdd, qd_des, taum,...
              taucomm, tauPD, tauHighlevel, massMatrix, S_transposed] = read_parameters(bag, name)
          
%   Read parameters from the rosbag

    %% nonlinear and inertial terms
    % nonlinear terms
    nonlinearTerms_bag = select(bag,'Topic',strcat(name,'/nonlinear_terms'));
    nonlinearTerms_struct = readMessages(nonlinearTerms_bag,'DataFormat','struct');

    % generalized velocities
    genVelocities_bag = select(bag,'Topic',strcat(name,'/gen_velocities'));
    genVelocities_struct = readMessages(genVelocities_bag,'DataFormat','struct');
    
    % generalized velocities desired
    genVelocitiesDes_bag = select(bag,'Topic',strcat(name,'/gen_velocities_des'));
    genVelocitiesDes_struct = readMessages(genVelocitiesDes_bag,'DataFormat','struct');

    % generalized coordinates
    genCoordinates_bag = select(bag,'Topic',strcat(name,'/gen_coordinates'));
    genCoordinates_struct = readMessages(genCoordinates_bag,'DataFormat','struct');
    
    % joint torques
    jointTorques_bag = select(bag,'Topic',strcat(name,'/joint_torques'));
    jointTorques_struct = readMessages(jointTorques_bag,'DataFormat','struct');
    
    % alma c highlevel controller torques
    highlevelTorques_bag = select(bag,'Topic',strcat(name,'/joint_torques_highlevelctrl'));
    highlevelTorques_struct = readMessages(highlevelTorques_bag,'DataFormat','struct');

    % actuator commands torques
    pdcommandTorques_bag = select(bag,'Topic',strcat(name,'/joint_torques_commanded'));
    pdcommandTorques_struct = readMessages(pdcommandTorques_bag,'DataFormat','struct');

    % mass matrix
    massMatrix_bag = select(bag,'Topic',strcat(name,'/mass_matrix'));
    massMatrix_struct = readMessages(massMatrix_bag,'DataFormat','struct');

    %% time
    time = nonlinearTerms_bag.MessageList.Time;
    time = time - time(1,1);
    endInd = size(time,1)-20; 
    % if some data sets have less samples recorded, this accounts for it
    time = time(1:endInd);
    % for low-pass filter the time vector needs to have equal spacing
    timeVec = linspace(0, time(endInd), endInd); 
    % sampling time [sec]
    Ts = time(endInd)/endInd; 

    %% save in vectors and matrices
    nonlinearTerms = zeros(24,endInd);
    q = zeros(24,endInd);
    qd = zeros(24,endInd);
    qd_des = zeros(18,endInd);
    taum = zeros(18,endInd);
    taucomm = zeros(18,endInd);
    tauPD = zeros(18,endInd);
    tauHighlevel = zeros(18,endInd);
    massMatrix = cell(1,endInd);
    for i = 1:endInd
      nonlinearTerms(:,i) = nonlinearTerms_struct{i,1}.Data(:,1);
      q(:,i) = genCoordinates_struct{i,1}.Data(:,1);
      qd(:,i) = genVelocities_struct{i,1}.Data(:,1);
      qd_des(:,i) = genVelocitiesDes_struct{i,1}.Data(:,1);
      taum(:,i) = jointTorques_struct{i,1}.Data(:,1);
      taucomm(:,i) = pdcommandTorques_struct{i,1}.Data(:,1) +  highlevelTorques_struct{i,1}.Data(:,1);
      tauPD(:,i) = pdcommandTorques_struct{i,1}.Data(:,1);
      tauHighlevel(:,i) = highlevelTorques_struct{i,1}.Data(:,1); 
      massMatrix{1,i} = zeros(24,24);          
      for j = 1:24
          massMatrix{1,i}(j,1:24) = massMatrix_struct{i,1}.Data((24*(j-1)+1):(24*j));  
      end
    end

    % compute acceleration
    qdd = diff(qd,1,2);
    
    % actuator selection matrix
    S_transposed = zeros(24,18);
    S_transposed(7:24,1:18) = eye(18,18);

end

