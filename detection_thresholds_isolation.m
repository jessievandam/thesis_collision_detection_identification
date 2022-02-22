%% open rosbag

% recordings 2-12: ocs2 ee
% bag = rosbag('arm_push_home.bag');
% bagft = rosbag('arm_push_home_ft.bag');
bag = rosbag('arm_push_forward.bag');
bagft = rosbag('arm_push_forward_ft.bag');
% bag = rosbag('arm_push_moving.bag');
% bagft = rosbag('arm_push_moving_ft.bag');
% bag = rosbag('base_push.bag');
% bagft = rosbag('base_push_ft.bag');

% recordings 15-12: armctrljointspace
% bag = rosbag('armCtrlJoint_push.bag');
% bagft = rosbag('armCtrlJoint_push_ft.bag');
% bag = rosbag('armCtrlJoint_push_move.bag');
% bagft = rosbag('armCtrlJoint_push_move_ft.bag');
% bag = rosbag('armCtrlJoint_push_load.bag');
% bagft = rosbag('armCtrlJoint_push_load_ft.bag');

% recordings 21-12: armctrljointspace
% bag = rosbag('armctrl_armload_push.bag');
% bagft = rosbag('armctrl_armload_push_ft.bag');
% bag = rosbag('armctrl_baseload_push.bag');
% bagft = rosbag('armctrl_baseload_push_ft.bag');
% bag = rosbag('armctrl_baseload_push2kg.bag');
% bagft = rosbag('armctrl_baseload_push2kg_ft.bag');
% bag = rosbag('armctrl_basepush.bag');
% bagft = rosbag('armctrl_basepush_ft.bag');
% bag = rosbag('armctrl_grabbing_push.bag');
% bagft = rosbag('armctrl_grabbing_push_ft.bag');
% bag = rosbag('armctrl_grabbing_push2.bag');
% bagft = rosbag('armctrl_grabbing_push2_ft.bag');
% bag = rosbag('armctrl_grabbing_push_move.bag');
% bagft = rosbag('armctrl_grabbing_push_move_ft.bag');
% bag = rosbag('armctrl_push_andreea.bag');
% bagft = rosbag('armctrl_push_andreea_ft.bag');
% bag = rosbag('armctrl_push_human.bag');

% recording 24-01
% bag = rosbag('armctrljoint_basepush_armmove.bag');
% bagft = rosbag('armctrljoint_basepush_armmove_ft.bag');
% bag = rosbag('armctrljoint_grab.bag');
% bagft = rosbag('armctrljoint_grab_ft.bag');
% bag = rosbag('armctrljoint_maria.bag');
% bagft = rosbag('armctrljoint_maria_ft.bag');
% bag = rosbag('ocs2base_trot.bag');
% bagft = rosbag('ocs2base_trot_ft.bag');
% bag = rosbag('ocs2ee_basepush_armmove.bag');
% bagft = rosbag('ocs2ee_basepush_armmove_ft.bag');

%% load data set specific parameters, computed in 'ftsensor_calibration.m'

% recordings 2-12
% load('arm_push_home');
load('arm_push_forward');
% load('arm_push_moving');
% load('base_push');

% recordings 15-12
% load('armCtrlJoint_push');
% load('armCtrlJoint_push_move');
% load('armCtrlJoint_push_load');

% recordings 21-12
% load('armctrl_armload_push');
% load('armctrl_baseload_push');
% load('armctrl_baseload_push2kg');
% load('armctrl_basepush');
% load('armctrl_grabbing_push');
% load('armctrl_grabbing_push2');
% load('armctrl_grabbing_push_move');
% load('armctrl_push_andreea');

% recordings 24-01
% load('armctrljoint_basepush_armmove');
% load('armctrljoint_grab');
% load('armctrljoint_maria');
% load('ocs2base_trot');
% load('ocs2ee_basepush_armmove');

%% read jacobians

% for dataset 24-01: '/collision_detection'; else: 'ocs2_controller_collision'
datasetName = '/ocs2_controller_collision';

% order allJacobians: gripper, wrist2, wrist1, forearm, upperarm, base
[allJacobians, jacobiansFeet_struct] = read_jacobians(bag, datasetName);

%% read parameters

[time, endInd, timeVec, Ts, nonlinearTerms, ~, qd, ~, qd_des, taum,...
  ~, ~, ~, massMatrix, S_transposed] = read_parameters(bag, datasetName);
  
%% read values FT sensor collision: ground truth force
% shift time and force vector to synchronize with wrench estimation data

[timeftShifted, ftForceCol, magFTForce] = read_ft_collision(bagft,n1sec,n2sec,timeShift);

%% read torques MBO used for tuning collision detection

% tuned gains external observers
[K_O, ~, ~, ~, ~, ~] = read_tuned_params(Ts);

% MBO
torques = momentum_observer(taum, nonlinearTerms, massMatrix, qd, endInd, Ts, S_transposed, K_O);

%% estimate force base and arm (wrist1)

% cut-off frequency LPF wrench, to be applied when arm is moving
fc = 1.0; % [Hz] 
 
% estimate force: first entry is arm force, second entry is base force
[force, forceLPF, magEstForce, magEstForceLPF] = estimate_base_and_arm_force_detection(torques,...
                            allJacobians, jacobiansFeet_struct, endInd, fc, timeVec);

%% read values FT sensor EE
% only works for recordings 21-12

[ftForceEE, ftForceEELPF, magFTForceEE, magFTForceEElpf, forceMinusEE] =...
    read_ft_ee(bag,endInd, time, timeVec, force);

%% time domain filtering: add BPF to forces

% parameters BPF 
cutOffFreqMin = 0.2;  % [Hz] HPF cut-off freq
cutOffFreqMax = 1.5;  % [Hz] LPF cut-off freq

% normal
[forceBPF, forceHPF] = filtering_time_domain(cutOffFreqMin,cutOffFreqMax,force, timeVec, endInd);
% EE force subtraction
% [forceBPF, forceHPF] = filtering_time_domain(cutOffFreqMin,cutOffFreqMax,forceMinusEE, timeVec, endInd);

%% tune constant threshold

% constant threshold 
% for comparison threshold, only the motion values are used
% constValues = [1.2; 1.2; 1.2];  % stance
constValues = [4; 1.4; 6.7];    % motion
threshConst = constValues .* ones(3,endInd);

%% compute and tune dynamic threshold based on velocity

% cut off frequency LPF [Hz] 
fc = 2.0; 

% dynamic threshold qd arm
staticThresh = [2.0; 1.1; 3.0].* ones(3,endInd); % armCtrlJoint_push_move + arm_push_moving + final
K = 2 * ones(3,18);
K(:,12:18) = 5;   % more importance on arm velocities during arm move
K(3,12:18) = 20;  % z force is more responsive and thus higher weights
K = K*0.07;       % for arm_push_moving, arm_push_home, base_push ocs2ee: MPC controller requires lower gains
[dynamicThreshQd, dynamicThreshQdLPF] = dynamic_threshold_velocity...
            (staticThresh, K, qd_des, endInd, fc, timeVec);
                        
%% compute and tune dynamic threshold based on STD of a certain window

% window size: time domain
windowSizeTime = 1.0;                 % [sec]
windowSize = ceil(windowSizeTime/Ts); % [number of samples]

% maximum amount of force that is applied based on sigma value
Fstd = [3;3;4];    % [N]

% constant gain
c1 = [45;35;20]; 

% cut off frequency LPF [Hz]
fc = 3.0;

% dynamic threshold std arm
staticThresh = [1.0; 0.7; 3.0].* ones(3,endInd);   
[dynamicThreshSTD, dynamicThreshSTDlpf] = dynamic_threshold_std(endInd,...
            force, c1, Fstd, windowSize, staticThresh, fc, timeVec);

%% plot filtered forces constant vs dynamic threshold

% choose params
T_twopeaks = 1.6;
T_rippling = 0.6;
jacobian = 1; % choose 1 for arm jacobian and 2 for base

% detect collision
% detect collision constant threshold
[collisionConst] = detection_time(forceBPF{jacobian,1},threshConst,endInd,Ts, T_twopeaks, T_rippling);
% detect collision dynamic velocity threshold
[collisionDynQd] = detection_time(forceBPF{jacobian,1},dynamicThreshQdLPF,endInd,Ts, T_twopeaks, T_rippling);
% detect collision dynamic STD threshold
[collisionDynSTD] = detection_time(forceBPF{jacobian,1},dynamicThreshSTDlpf{jacobian,1},endInd,Ts, T_twopeaks, T_rippling);

% plot full time interval
startTime = nnsec;
endTime = mmsec;

% choose which filtered force and which dynamic and constant threshold to plot
constThresh = threshConst; 
threshDyn = dynamicThreshSTDlpf{jacobian,1}; % or substitute with dynamicThreshQdLPF
forceFiltered = forceBPF{jacobian,1}; 
plot_detection_time_domain(magFTForce,timeftShifted,forceFiltered, time,...
                            startTime, endTime, constThresh, threshDyn, collisionDynSTD);
                        
%% compute delay

minTimeCol = 2;
minHeightCol = 7;

[timesCollisionCutFinalConst,timesFTCutFinal, delayConst, delayAvConst] = compute_detection_delay_detection...
                (time,timeftShifted,collisionConst,endInd,magFTForce, minTimeCol,...
                minHeightCol, nnsec, mmsec, Ts); 

[timesCollisionCutFinalDynQd,timesFTCutFinal, delayDynQd, delayAvDynQd] = compute_detection_delay_detection...
                (time,timeftShifted,collisionDynQd,endInd,magFTForce, minTimeCol,...
                minHeightCol, nnsec, mmsec, Ts); 
 
[timesCollisionCutFinalDynSTD,timesFTCutFinal, delayDynSTD, delayAvDynSTD] = compute_detection_delay_detection...
        (time,timeftShifted,collisionDynSTD,endInd,magFTForce, minTimeCol,...
        minHeightCol, nnsec, mmsec, Ts);     

%% compute how much percentage of the total collision force the force has risen after detection

[forceRiseConst, forceRiseAvConst] = compute_force_rise(timesCollisionCutFinalConst,...
            timesFTCutFinal,magFTForce, time, timeftShifted);

[forceRiseDynQd, forceRiseAvDynQd] = compute_force_rise(timesCollisionCutFinalDynQd,...
            timesFTCutFinal,magFTForce, time, timeftShifted);
        
[forceRiseDynSTD, forceRiseAvDynSTD] = compute_force_rise(timesCollisionCutFinalDynSTD,...
            timesFTCutFinal,magFTForce, time, timeftShifted);

%% collision isolation: plot torques to show that no distinction can be made between upperarm and forearm

% add BPF to torques, input from Hz to rad/sec
cutOffFreqMin = 0.4;
cutOffFreqMax = 3.0;
sysBPF = tf([0.0, cutOffFreqMax*2*pi, 0.0], [1.0, (cutOffFreqMax*2*pi+cutOffFreqMin*2*pi), (cutOffFreqMax*cutOffFreqMin*4*pi*pi)]);
torquesBPF = zeros(24, endInd);
for i = 1:24
    % normal
    torquesBPF(i,:) = lsim(sysBPF,torques(i,:),timeVec);
    % with object in gripper
%     torquesBPF(i,:) = lsim(sysBPF,torques(i,:)-torquesEE(i,:),timeVec);
end

% plot full time interval
startTime = nnsec;
endTime = mmsec;

% joints to plot
j1 = 19;
j2 = 20;
j3 = 21;

% forearm/upperarm isolation
% thresh = [1.1; 4.5; 0.8].* ones(3,endInd); % PID
thresh = [1.1; 4.5; 1.8].* ones(3,endInd); % MPC

% base/arm isolation
% thresh = [1.3; 3.5; 1.7].* ones(3,endInd);
plot_torques(time, timeftShifted, abs(torquesBPF), magFTForce, thresh, j1, j2, j3, startTime, endTime);

% joints to plot
j1 = 22;
j2 = 23;
j3 = 24;

% forearm/upperarm isolation
% thresh = [0.12; 0.41; 0.23].* ones(3,endInd); % PID
thresh = [0.11; 0.41; 0.23].* ones(3,endInd); % MPC

% base/arm isolation
% thresh = [0.075; 0.95; 0.16].* ones(3,endInd); 
plot_torques(time, timeftShifted, abs(torquesBPF), magFTForce, thresh, j1, j2, j3, startTime, endTime);
