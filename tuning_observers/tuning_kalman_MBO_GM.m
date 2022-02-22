% include folder wrench_estimation for datasets and function files

% recordings 15-12: armctrljointspace
% open rosbag
% bag = rosbag('armCtrlJoint_push.bag');
% bagft = rosbag('armCtrlJoint_push_ft.bag');
% bag = rosbag('armCtrlJoint_push_move.bag');
% bagft = rosbag('armCtrlJoint_push_move_ft.bag');
% bag = rosbag('armCtrlJoint_push_load.bag');
% bagft = rosbag('armCtrlJoint_push_load_ft.bag');
% load data set specific parameters, computed in 'ftsensor_calibration.m'
% load('armCtrlJoint_push');
% load('armCtrlJoint_push_move');
% load('armCtrlJoint_push_load');

% recordings 21-12: armctrljointspace
% bag = rosbag('armctrl_basepush.bag');
% bagft = rosbag('armctrl_basepush_ft.bag');
% load('armctrl_basepush');

bag = rosbag('arm_push_forward.bag');
bagft = rosbag('arm_push_forward_ft.bag');
load('arm_push_forward');

%% read parameters

% for dataset 24-01: '/collision_detection'; else: 'ocs2_controller_collision'
datasetName = '/ocs2_controller_collision';

[time, endInd, timeVec, Ts, nonlinearTerms, q, qd, qdd, qd_des, taum,...
  taucomm, tauPD, tauHighlevel, massMatrix, S_transposed] = read_parameters(bag, datasetName);
 
%% read values FT sensor collision: ground truth force
% shift time and force vector to synchronize with wrench estimation data

[timeftShifted, ftForceCol, magFTForce] = read_ft_collision(bagft,n1sec,n2sec,timeShift);

%% read and save torques for comparison

% stack in cell
torquesStacked = cell(3,1);
for k = 1:3
    torquesStacked{k,1} = zeros(24,endInd);
end

%% tune MBO and GM

% MBO: gain matrix [s^-1]
% tuned s.t. all std noise is between 0 and 0.4 Nm
gains = [15, 15, 3, 15, 3, 15,... % base, last 3 terms could be set lower % base z pose and y torque more noise in std, thus lower terms       
        40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,... % legs
        10, 10, 10, 10, 10, 10];  % arm % translational joints lower to reduce noise ? 
K_O = diag(gains); 

% GM: cut-off frequency [Hz]
FC = exp(-Ts*gains);

torquesStacked{1,1} = gm_observer(taum, nonlinearTerms, massMatrix, qd, endInd, Ts, S_transposed, FC);
torquesStacked{2,1} = momentum_observer(taum, nonlinearTerms, massMatrix, qd, endInd, Ts, S_transposed, K_O);

%% tune Kalman

% parameters
N = 24;
    
% matrices to tune
Af = zeros(N,N);   % determines dynamics external wrench: set to negative diagonal to mitigate offset
Qp = 0.1*eye(N,N); % process noise in joint friction, the larger the weight, the more friction uncertainty
Qf = 500*eye(N,N); % process noise in contact forces and torques (example: 4x higher than Qp). the larger the weights,
                   % the less kalman filter will rely on fdot = 0 (contact forces being constant), however, also
                   % the larger noise amplification            

% larger weight = smoother signal, but more delay
% base: a lot of parametric uncertainty (inertial terms)
% the higher, the less the ripples, but the higher the overshoot peaks
Qp(1:6,1:6) = Qp(1:6,1:6)*800; 
% legs: second joint shows more friction uncertainty
Qp(8,8) = Qp(8,8)*10;
Qp(11,11) = Qp(11,11)*10;
Qp(14,14) = Qp(14,14)*10;
Qp(17,17) = Qp(17,17)*10;
% arm: more friction uncertainty than legs
Qp(19:24,19:24) = Qp(19:24,19:24)*50; 

% base: faster response required
Qf(1:6,1:6) = Qf(1:6,1:6)*10; % decreases ripples after collision and overshoot peaks
% arm: also faster response desired, but since arm is shaky it causes too
% much overshoot and random peaks

%% compute torques with kalman momentum based observer

torquesStacked{3,1} = kalman_observer(taum, nonlinearTerms,...
                    massMatrix, qd, endInd, Ts, S_transposed, Af, Qp, Qf, N);

%% plot torques to compare response and tune covariance matrices

% torques to plot: 1 to 24 (count base as joint as well)
j1 = 1;
j2 = 2;
j3 = 3;
torques = torquesStacked{3,1}; % choose method

plot_torques(time, timeftShifted, torques, magFTForce, [], j1, j2, j3, nnsec, mmsec);

%% after tuning torques: read jacobians

% order allJacobians: gripper, wrist2, wrist1, forearm, upperarm, base
[allJacobians, jacobiansFeet_struct] = read_jacobians(bag, datasetName);

%% after tuning torques: compute force

force = cell(3,1);
for k = 1:3
    for i = 1:endInd

        % read out torques calculated by MBO, GM, direct
        torques = torquesStacked{k,1}(:,i);

        % colliding body jacobian
        jacobianColBody = allJacobians{3,1}{i,1}.Data(1:end); % wrist1 jacobian
        jacobiansFeet = jacobiansFeet_struct{i,1}.Data(1:end); 
        for j = 1:12
            stackedFeetJacobian(j,:) = jacobiansFeet((24*(j-1)+1):(24*j));
        end
         stackedJacobian = [stackedFeetJacobian; 
                        jacobianColBody(1:24)'; jacobianColBody(25:48)'; jacobianColBody(49:72)'; 
                        jacobianColBody(73:96)'; jacobianColBody(97:120)'; jacobianColBody(121:144)'];

        % estimate stacked force vector feet and force,moment colliding body
        wrench = pinv(stackedJacobian')*torques; 
        force{k,1}(:,i) = wrench(13:15); 
    end
end

magEstForce = cell(3,1);
for k = 1:3 % loop over all external torques
   magEstForce{k,1} = sqrt(force{k,1}(1,:).^2 + force{k,1}(2,:).^2 + force{k,1}(3,:).^2);   
end

%% plot forces

% figure params
LW = 1;
FS = 20;
x00=10;
y00=10;
width=1000;
height=1400;

% synchronize time interval (since different time steps are used) for estimated and ground truth force
n = find(time>0,1);
m1 = find(time>(min(time(end), timeftShifted(end))-1),1);          % estimated force endInd
m2 = find(timeftShifted>(min(time(end), timeftShifted(end))-1),1); % ft sensor collision endInd

% time to be plotted
nn = find(time>nnsec,1); 
mm = find(time>mmsec,1);    

% plot min and max of y axes
figure()
set(gcf,'position',[x00,y00,width,height/2])
plot(time(n:m1), magEstForce{1,1}(1,n:m1)-12, 'b', 'LineWidth', LW)
hold on
plot(time(n:m1), magEstForce{2,1}(1,n:m1)-12, 'r', 'LineWidth', LW)
hold on
plot(time(n:m1), magEstForce{3,1}(1,n:m1)-12, 'g', 'LineWidth', LW)
hold on
plot(timeftShifted(n:m2), magFTForce(1,n:m2)-2, 'k', 'LineWidth', LW)
hold on
hold off
grid on
title('Magnitude arm force','Interpreter','latex','Fontsize', FS)
ylabel('Force [N]','Interpreter','latex','Fontsize', FS)
xlabel('Time [sec]','Interpreter','latex','Fontsize', FS)
leg = legend('GM', 'MBO', 'Kalman', 'FT');
set(leg, 'Location', 'southeast',  'Interpreter', 'latex','Fontsize', 18);
xlim([time(nn) time(mm)])
% exportgraphics(gcf, 'fig_armctrl_push_human_col1to6.png', 'Resolution', 300);

