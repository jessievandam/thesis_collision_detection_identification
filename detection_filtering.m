%% open rosbag

% recordings 2-12: ocs2 ee
bag = rosbag('arm_push_home.bag');
bagft = rosbag('arm_push_home_ft.bag');
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

%% load data set specific parameters, computed in 'ftsensor_calibration.m'

% recordings 2-12
load('arm_push_home');
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
% load('armctrl_basepush');
% load('armctrl_grabbing_push');
% load('armctrl_grabbing_push2');
% load('armctrl_grabbing_push_move');
% load('armctrl_push_andreea');

% recordings 24-01
load('armctrljoint_basepush_armmove');
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
[K_O, ~, ~, ~, ~, ~, ~, ~, ~] = read_tuned_params(Ts);
   
% MBO
torques = momentum_observer(taum, nonlinearTerms, massMatrix, qd, endInd, Ts, S_transposed, K_O);

%% estimate force base and arm

% cut-off frequency LPF wrench, to be applied when arm is moving
fc = 1.0; % [Hz] 
 
% estimate force: first entry is arm force, second entry is base force
[force, forceLPF, magEstForce, magEstForceLPF] = estimate_base_and_arm_force_detection(torques,...
                            allJacobians, jacobiansFeet_struct, endInd, fc, timeVec);

%% read values FT sensor EE
% only works for recordings 21-12 and 24-01

[ftForceEE, ftForceEELPF, magFTForceEE, magFTForceEElpf] = read_ft_ee(bag,n3sec,n4sec, endInd, time, timeVec);

forceMinusEE = cell(2,1);
for h = 1:2 % loop over base and arm jacobian
    for i = 1:endInd  
        forceMinusEE{h,1}(:,i) = force{h,1}(:,i)-ftForceEE(:,i);
    end
end
                     
%% frequency domain filtering: FFT and Hann-window

% initialize
forceFFT = cell(2,1);
forceFFTlim = cell(2,1);
forceFFThann = cell(2,1);
forceFFThannlim = cell(2,1);

% time window size nfft to convert force from time to frequency domain
% freq/nfft = freq res = 1/(time resolution)
% Lower freq resolution gives higher time resolution gives wider responses
freqRes = 0.5; % desired resolution [Hz]

% filtering frequency domain: cut-off frequencies min (f1, f3) and max (f2, f4)
% f*freqRes = cut-off frequency in Hz
f1 = 2;   % min for FFT
f2 = 4;   % max for FFT
f3 = 2;   % min for FFT + hann
f4 = 4;   % max for FFT + hann

% normal
[forceFFT{1,1}, forceFFTlim{1,1}, forceFFThann{1,1}, forceFFThannlim{1,1}, freqResReal, timeResReal, windowSize] = ...
                    filtering_frequency_domain(freqRes, Ts, force{1,1}, f1, f2, f3, f4, endInd);
% EE force subtraction
% [forceFFT{1,1}, forceFFTlim{1,1}, forceFFThann{1,1}, forceFFThannlim{1,1}, freqResReal, timeResReal, windowSize] = ...
%                     filtering_frequency_domain(freqRes, Ts, forceMinusEE{1,1}, f1, f2, f3, f4, endInd);

%% plot raw and filtered forces to tune freq domain filtering

% plot full time interval
startTime = nnsec;
endTime = mmsec;
plot_filtering_freq_domain(magFTForce,timeftShifted,...
                    forceFFTlim{1,1}, forceFFThannlim{1,1},[] , [],...
                    'DFT lim', 'DFT Hann lim', '', '',...
                    'Arm', time, startTime, endTime);

%% time domain filtering: add BPF to forces

% initialize
forceBPF = cell(2,1);
forceHPF = cell(2,1);

% parameters BPF 
cutOffFreqMin = freqResReal;  % [Hz] HPF cut-off freq
cutOffFreqMax = freqResReal*4;  % [Hz] LPF cut-off freq

% normal
[forceBPF{1,1}, forceHPF{1,1}] = filtering_time_domain(cutOffFreqMin,cutOffFreqMax,force{2,1}, timeVec, endInd);
% EE force subtraction
% [forceBPF{1,1}, forceHPF{1,1}] = filtering_time_domain(cutOffFreqMin,cutOffFreqMax,forceMinusEE{1,1}, timeVec, endInd);

%% plot raw and filtered forces to tune time domain filtering

% plot full time interval
startTime = nnsec;
endTime = mmsec;
plot_filtering_time_domain(magFTForce, timeftShifted, force{1,1}, forceHPF{1,1},...
                         forceBPF{1,1}, 'unfiltered', 'HPF', 'BPF',...
                         'Arm', time, startTime, endTime)
                     
%% plot filtered forces in frequency domain with constant threshold

constValues = [2.15; 0.8; 4.1];    
threshConst= constValues .* ones(3,endInd);

% detect collision
T_rippling = 0.2;
[collisionFreq] = detection_freq_hann(forceFFThannlim{1,1},threshConst,...
                    endInd,Ts, T_rippling);

% plot full time interval
startTime = nnsec;
endTime = mmsec;

% choose which filtered force and which dynamic and constant threshold to plot
constThresh = threshConst;    
forceFiltered = forceFFThannlim{1,1};
plot_detection_freq_domain(magFTForce,timeftShifted,forceFiltered, time,...
                            startTime, endTime, constThresh, collisionFreq);

%% plot filtered forces in time domain with constant threshold

constValues = [2.55; 0.95; 5.25];   
threshConst = constValues .* ones(3,endInd);

% detect collision
T_rippling = 0.6;
T_twopeaks = 1.5; 
[collisionTime] = detection_time(forceBPF{1,1},threshConst, endInd,...
                Ts, T_twopeaks, T_rippling,time);

% plot full time interval
startTime = nnsec;
endTime = mmsec;

% choose which filtered force and which dynamic and constant threshold to plot
constThresh = threshConst;     
forceFiltered = forceBPF{1,1};
plot_detection_time_domain(magFTForce,timeftShifted,forceFiltered, time,...
                            startTime, endTime, constThresh, collisionTime);

%% compute delay

minTimeCol = 2;
minHeightCol = 6;

[timesCollisionCutFinalFreq,timesFTCutFinal, delayFreq, delayAvFreq] = compute_detection_delay_detection(time,...
                timeftShifted,collisionFreq,endInd,magFTForce, minTimeCol, minHeightCol, nnsec,...
                mmsec, Ts);

[timesCollisionCutFinalTime,timesFTCutFinal, delayTime, delayAvTime] = compute_detection_delay_detection(time,...
                timeftShifted,collisionTime,endInd,magFTForce, minTimeCol, minHeightCol, nnsec,...
                mmsec, Ts);