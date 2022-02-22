%% open rosbag

% recordings 2-12: ocs2 ee
% bag = rosbag('arm_push_home.bag');
% bagft = rosbag('arm_push_home_ft.bag');
% bag = rosbag('arm_push_forward.bag');
% bagft = rosbag('arm_push_forward_ft.bag');
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
bag = rosbag('armctrl_basepush.bag');
bagft = rosbag('armctrl_basepush_ft.bag');
% bag = rosbag('armctrl_grabbing_push_short.bag');
% bagft = rosbag('armctrl_grabbing_push_ft.bag');
% bag = rosbag('armctrl_grabbing_push2.bag');
% bagft = rosbag('armctrl_grabbing_push2_ft.bag');
% bag = rosbag('armctrl_grabbing_push_move.bag');
% bagft = rosbag('armctrl_grabbing_push_move_ft.bag');
% bag = rosbag('armctrl_push_andreea.bag');
% bagft = rosbag('armctrl_push_andreea_ft.bag');
% bag = rosbag('armctrl_push_human.bag');
   
% recordings 24-01: armctrljointspace & ocs2
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

%% load data set specific parameters, computed in 'calibration.m'

% recordings 2-12
% load('arm_push_home');
% load('arm_push_forward');
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
load('armctrl_basepush');
% load('armctrl_grabbing_push');
% load('armctrl_grabbing_push2');
% load('armctrl_grabbing_push_move');
% load('armctrl_push_andreea');
% load('armctrl_push_human');

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

[time, endInd, timeVec, Ts, nonlinearTerms, q, qd, qdd, ~, taum,...
  taucomm, tauPD, tauHighlevel, massMatrix, S_transposed] = read_parameters(bag, datasetName);

%% read values FT sensor collision: ground truth force
% shift time and force vector to synchronize with wrench estimation data

[timeftShifted, ftForceCol, magFTForce] = read_ft_collision(bagft,n1sec,n2sec,timeShift);

%% read and save torques

% stack in cell
torquesStacked = cell(4,1);
for k = 1:4
    torquesStacked{k,1} = zeros(24,endInd);
end

% tuned gains external observers
[K_O, FC, N, Af, Qp, Qf, K_O1, K_O2, K_O3] = read_tuned_params(Ts);
                 
% external observers: comparison all methods
% torquesStacked{1,1} = direct_estimation(taum, nonlinearTerms, massMatrix, qdd, endInd, S_transposed, timeVec);
% torquesStacked{2,1} = motor_error(taum, nonlinearTerms, endInd, S_transposed);
% torquesStacked{3,1} = momentum_observer(taum, nonlinearTerms, massMatrix, qd, endInd, Ts, S_transposed, K_O);
% torquesStacked{4,1} = gm_observer(taum, nonlinearTerms, massMatrix, qd, endInd, Ts, S_transposed, FC);
% torquesStacked{5,1} = momentum_observer_higher_order(taum, nonlinearTerms, massMatrix,...
%                         qd, endInd, Ts, S_transposed, K_O1, K_O2, K_O3);                   
% torquesStacked{6,1} = kalman_observer(taum, nonlinearTerms, massMatrix, qd, endInd, Ts, S_transposed, Af, Qp, Qf, N);

% external observers: comparison method paper 4 best estimators: the 4th is to be used for our proposed identification method
torquesStacked{1,1} = momentum_observer(taum, nonlinearTerms, massMatrix, qd, endInd, Ts, S_transposed, K_O);
torquesStacked{2,1} = gm_observer(taum, nonlinearTerms, massMatrix, qd, endInd, Ts, S_transposed, FC);
torquesStacked{3,1} = momentum_observer_higher_order(taum, nonlinearTerms, massMatrix,...
                        qd, endInd, Ts, S_transposed, K_O1, K_O2, K_O3);                   
torquesStacked{4,1} = momentum_observer(taum, nonlinearTerms, massMatrix, qd, endInd, Ts, S_transposed, K_O); 

%% estimate force colliding body and its magnitude 

% cut-off frequency LPF force, to be applied during trotting
fc = 1.0; % [Hz] 
 
[force, forceLPF, magEstForce, magEstForceLPF, moment] = estimate_force_base_arm(torquesStacked,...
                allJacobians, jacobiansFeet_struct, endInd, fc, timeVec);

%% read values FT sensor EE for manipulation task integration and subtract from estimated force
% only to be used with datasets 21-12 & 24-01 since other FT EE are not recorded well

[ftForceEELPF, forceMinusFTEE, magEstForceMinusFTEE, magEstForceLPFMinusFTEE] =...
                    subtract_ft_ee(bag, magEstForce, magEstForceLPF, force, ...
                    endInd, timeVec, n3sec, n4sec, datasetName);

%% use simple collision detection to decide if collision occurs

% parameters for collision detection flowchart
T_twopeaks = 2.5;   
T_rippling = 0.4; 

% cut-off frequencies of BPF
cutOffFreqMin = 0.4;  % [Hz] HPF cut-off freq
cutOffFreqMax = 3.0;  % [Hz] LPF cut-off freq
% trotting
%     cutOffFreqMin = 0.1;  % [Hz] HPF cut-off freq
%     cutOffFreqMax = 1.5;  % [Hz] LPF cut-off freq

% constant threshold
constThresh = [1.8; 1.8; 1.8]; % stance
%    constThresh = [4; 3; 6.5]; % arm motion 
%    constThresh = [15.8; 15.6; 7.9]; % trotting

[collision, forceBPF, forceHPF] = collision_detection(force, timeVec, endInd, nnsec, mmsec,...
                                    time, Ts, T_twopeaks, T_rippling, cutOffFreqMin, cutOffFreqMax, constThresh);

% when grabbing object: dataset 21-12 & 24-01
% [collision, forceBPF, forceHPF] = collision_detection(forceMinusFTEE, timeVec, endInd, nnsec, mmsec,...
%                                      time, Ts, T_twopeaks, T_rippling, cutOffFreqMin, cutOffFreqMax, constThresh);
    
%% constantly subtract offset of 1 timestep ago, and when collision occurs fix offset from T_col timesteps ago

% LPF force used during trotting
[magEstForceFinal, magEstForceLPFfinal, offset] = collision_identification(collision,...
                magEstForce, magEstForceLPF, endInd, Ts);
            
% when grabbing object: dataset 21-12 & 24-01
% [magEstForceFinal, magEstForceLPFfinal, offset] = collision_identification(collision,...
%                 magEstForceMinusFTEE, magEstForceLPFMinusFTEE, endInd, Ts);

%% force magnitudes, comparing various external observers or jacobians

% figure params
LW = 1.5;
FS = 20;
x00=10;
y00=10;
width=1000;
height=1400;

% synchronize time interval (since different time steps are used) for estimated and ground truth force
n = find(time>0,1);
m1 = find(time>(min(time(end), timeftShifted(end))-1),1);          % estimated force endInd
m2 = find(timeftShifted>(min(time(end), timeftShifted(end))-1),1); % ft sensor collision endInd

% plot torques t and jacobian j
t1 = 1; 
t2 = 2; 
t3 = 3; 
t4 = 4; 
j1 = 1; % wrist1
j2 = 2; % base
  
% plot magnitude of collision force and detection bool
nn = find(time>nnsec,1); 
mm = find(time>mmsec,1);
torque = t1;
jacobian = j2;
figure()
set(gcf,'position',[x00,y00,width,height/2])
plot(time(n:m1), magEstForce{t4, jacobian}(1,n:m1), 'r', 'LineWidth', LW)
hold on
plot(time(n:m1), offset{t4, jacobian}(n:m1), 'm', 'LineWidth', LW)
hold on
plot(time(n:m1), magEstForceFinal{t4, jacobian}(1,n:m1), 'b', 'LineWidth', LW)
hold on
plot(time(n:m1), 10*collision{4,jacobian}(1,n:m1), 'r--', 'LineWidth', 2.0)
hold on
plot(timeftShifted(n:m2), magFTForce(1,n:m2), 'k', 'LineWidth', LW)
hold off
grid on
title('Magnitude force','Interpreter','latex','Fontsize', FS)
ylabel('Force [N]','Interpreter','latex','Fontsize', FS)
xlabel('Time [sec]','Interpreter','latex','Fontsize', FS)
leg = legend('Estimation', 'Disturbance', 'Final collision force','Collision?', 'Ground truth force');
set(leg, 'Location', 'northeast',  'Interpreter', 'latex','Fontsize', FS);
xlim([time(nn) time(mm)])   

%% plot BPF forces on x, y and z-axis to tune the collision detection thresholds in 'collision_detection'

% time to be plotted
startTime = nnsec; 
endTime = mmsec;   
 
plot_forces(magFTForce,[], [], timeftShifted, forceBPF{4,jacobian},...
                      'Arm filtered', time, startTime, endTime);

%% compute detection delay

% parameters: min time between collision peaks and min collision peak height
minTimeCol = 1.5;
minHeightCol = 3;

[timesCollisionCutFinal,timesFTCutFinal, delay, delayAv] = compute_detection_delay...
                (time,timeftShifted,collision,endInd,magFTForce, minTimeCol,...
                minHeightCol, nnsec, mmsec, Ts);
            
%% compare external observers based on certain metrics

% choose arm (1) or base (2) jacobian
jacobian = 1;

[overshoot, overshootAv, overEstimation, overEstimationAv, underEstimation, underEstimationAv,...
                   RMS, STD, estForcePeaksMin, estForcePeaksMinAv, FTpeaks, estForcePeaks] = ...
                   metrics_comparison(magEstForceFinal, magFTForce, magEstForce, time,...
                   timeftShifted, n, m1, m2, nnsec,timesFTCutFinal, jacobian);

%% compute time span collision

% choose jacobian for which to compare wrench 
jacobian = 1;
[timesCol,timesEstimation, error, accuracy, timesFTCutFinalStop] = compute_timespan(timesFTCutFinal,time,...
                    timesCollisionCutFinal, collision, jacobian, endInd, magFTForce, timeftShifted);               
           