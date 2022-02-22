% open rosbag
bag = rosbag('ocs2ee_basepush_armmove.bag');
bagft = rosbag('ocs2ee_basepush_armmove_ft.bag');

% script meant to find out:
% - n1/n2 : over which time interval offset FT sensor collision is computed
% - n3/n4 : over which time interval offset FT sensor EE is computed
% - timeShift : looking at start force both ft sensors and calculating diff
% - nn/mm : start and end time of time interval where collisions take place

%% read out FT sensor values collision and EE

% collision ground truth wrench
ftCollision_bag = select(bagft,'Topic','/rokubimini_cosmo/ft_sensor/wrench');
ftCollision_struct = readMessages(ftCollision_bag,'DataFormat','struct');

% EE wrench
% ftEE_bag = select(bag, 'Topic','/rokubimini_cosmo/ft_sensor/wrench'); 
% only for 24-01 recordings
ftEE_bag = select(bag, 'Topic','/collision_detection/ft_sensor_wrench_hw'); 
ftEE_struct = readMessages(ftEE_bag,'DataFormat','struct');

% time collision ft sensor
timeftCol = ftCollision_bag.MessageList.Time;
timeftCol = timeftCol - timeftCol(1,1);
endIndftCol = size(timeftCol,1);

% time EE ft sensor
time = ftEE_bag.MessageList.Time;
time = time - time(1,1);
endInd = size(time,1);

% read values
ftForceMeasCol = zeros(3,endIndftCol);
ftForceMeasEE = zeros(6,endInd);
ftForceMeasEEWrong = zeros(6,endInd);
for i = 1:endIndftCol
    ftForceMeasCol(1,i) = ftCollision_struct{i,1}.Wrench.Force.X;
    ftForceMeasCol(2,i) = ftCollision_struct{i,1}.Wrench.Force.Y;
    ftForceMeasCol(3,i) = ftCollision_struct{i,1}.Wrench.Force.Z;
end
for i = 1:endInd
%     ftForceMeasEE(1,i) = ftEE_struct{i,1}.Wrench.Force.X;
%     ftForceMeasEE(2,i) = ftEE_struct{i,1}.Wrench.Force.Y;
%     ftForceMeasEE(3,i) = ftEE_struct{i,1}.Wrench.Force.Z;
    
    % only for 24-01 recordings
    ftForceMeasEE(1,i) = ftEE_struct{i,1}.Force.X;
    ftForceMeasEE(2,i) = ftEE_struct{i,1}.Force.Y;
    ftForceMeasEE(3,i) = ftEE_struct{i,1}.Force.Z;
end

%% plot ft sensor forces EE and collision separately 
% to determine start time nn mm and time over which offset is computed n1 m1

% figure params
LW = 1;
FS = 20;
x00=10;
y00=10;
width=1000;
height=1400;

% full
nEE = 1;
mEE = endInd;
nCol = 1;
mCol = endIndftCol;

% time interval
% n = find(time>0.5,1);
% m = find(time>3.5,1);

figure()
set(gcf,'position',[x00,y00,width,height/2])

subplot(3,2,1)
plot(time(nEE:mEE), ftForceMeasEE(1,nEE:mEE), 'b', 'LineWidth', LW)
hold off
grid on
title('Force x EE','Interpreter','latex','Fontsize', FS)
ylabel('Force [N]','Interpreter','latex','Fontsize', FS)
xlim([time(nEE) time(mEE)])

subplot(3,2,2)
plot(timeftCol(nCol:mCol), ftForceMeasCol(1,nCol:mCol), 'b', 'LineWidth', LW)
hold off
grid on
title('Force x collision','Interpreter','latex','Fontsize', FS)
ylabel('Force [N]','Interpreter','latex','Fontsize', FS)
xlim([timeftCol(nCol) timeftCol(mCol)])

subplot(3,2,3)
plot(time(nEE:mEE), ftForceMeasEE(2,nEE:mEE), 'b', 'LineWidth', LW)
hold off
grid on
title('Force y EE','Interpreter','latex','Fontsize', FS)
ylabel('Force [N]','Interpreter','latex','Fontsize', FS)
xlim([time(nEE) time(mEE)])

subplot(3,2,4)
plot(timeftCol(nCol:mCol), ftForceMeasCol(2,nCol:mCol), 'b', 'LineWidth', LW)
hold off
grid on
title('Force y collision','Interpreter','latex','Fontsize', FS)
ylabel('Force [N]','Interpreter','latex','Fontsize', FS)
xlim([timeftCol(nCol) timeftCol(mCol)])

subplot(3,2,5)
plot(time(nEE:mEE), ftForceMeasEE(3,nEE:mEE), 'b', 'LineWidth', LW)
hold off
grid on
title('Force z EE','Interpreter','latex','Fontsize', FS)
ylabel('Force [N]','Interpreter','latex','Fontsize', FS)
xlim([time(nEE) time(mEE)])
xlabel('Time [sec]','Interpreter','latex','Fontsize', FS)

subplot(3,2,6)
plot(timeftCol(nCol:mCol), ftForceMeasCol(3,nCol:mCol), 'b', 'LineWidth', LW)
hold off
grid on
title('Force z collision','Interpreter','latex','Fontsize', FS)
ylabel('Force [N]','Interpreter','latex','Fontsize', FS)
xlim([timeftCol(nCol) timeftCol(mCol)])
xlabel('Time [sec]','Interpreter','latex','Fontsize', FS)

%% remove offset FT sensor collision
% time to compute mean offset ft sensors over

n1 = find(timeftCol>17,1);
n2 = find(timeftCol>21,1);

offset(1,1) = mean(ftForceMeasCol(1,n1:n2));
offset(2,1) = mean(ftForceMeasCol(2,n1:n2));
offset(3,1) = mean(ftForceMeasCol(3,n1:n2));

ftForceCol = zeros(3,endIndftCol);
for i = 1:endIndftCol
    ftForceCol(1,i) = ftForceMeasCol(1,i)-offset(1,1);
    ftForceCol(2,i) = ftForceMeasCol(2,i)-offset(2,1);
    ftForceCol(3,i) = ftForceMeasCol(3,i)-offset(3,1);
end

%% remove offset FT sensor EE
% time to compute mean offset ft sensors over

n3 = find(time>16,1);
n4 = find(time>20,1);
offset(1,1) = mean(ftForceMeasEE(1,n3:n4));
offset(2,1) = mean(ftForceMeasEE(2,n3:n4));
offset(3,1) = mean(ftForceMeasEE(3,n3:n4));

ftForceEE = zeros(3,endInd);
for i = 1:endInd
    ftForceEE(1,i) = ftForceMeasEE(1,i)-offset(1,1);
    ftForceEE(2,i) = ftForceMeasEE(2,i)-offset(2,1);
    ftForceEE(3,i) = ftForceMeasEE(3,i)-offset(3,1);
end

%% plot both ft sensor forces to synchronize timing

% figure params
LW = 1;
FS = 20;
x00=10;
y00=10;
width=1000;
height=1400;

% allign time vectors FT sensor data and estimated wrench
timeShift = -0.1183;
timeFTshifted = timeftCol + timeShift;

% part of interval where collisions take place
n = find(time>0,1);
m1 = find(time>(min(time(end), timeFTshifted(end))-1),1); 
m2 = find(timeFTshifted>(min(time(end), timeFTshifted(end))-1),1); 

% full interval 
nn = 1;
mm = min(endInd, endIndftCol);

% part of time interval
% first interval is with start and end calibration, second without
% nn = find(time>215,1); % time step to start plotting
% mm = find(time>233,1); % time step to stop plotting

figure()
set(gcf,'position',[x00,y00,width,height/2])

subplot(3,1,1)
plot(time(n:m1), ftForceEE(1,n:m1), 'b', 'LineWidth', LW)
hold on
plot(timeFTshifted(n:m2), ftForceCol(1,n:m2), 'r', 'LineWidth', LW)
hold off
grid on
title('Force x','Interpreter','latex','Fontsize', FS)
ylabel('Force [N]','Interpreter','latex','Fontsize', FS)
leg = legend('EE','collision');
set(leg, 'Location', 'eastoutside',  'Interpreter', 'latex','Fontsize', FS);
xlim([time(nn) time(mm)])

subplot(3,1,2)
plot(time(n:m1), ftForceEE(2,n:m1), 'b', 'LineWidth', LW)
hold on
plot(timeFTshifted(n:m2), ftForceCol(2,n:m2), 'r', 'LineWidth', LW)
hold off
grid on
title('Force y','Interpreter','latex','Fontsize', FS)
ylabel('Force [N]','Interpreter','latex','Fontsize', FS)
leg = legend('EE','collision');
set(leg, 'Location', 'eastoutside',  'Interpreter', 'latex','Fontsize', FS);
xlim([time(nn) time(mm)])

subplot(3,1,3)
plot(time(n:m1), ftForceEE(3,n:m1), 'b', 'LineWidth', LW)
hold on
plot(timeFTshifted(n:m2), ftForceCol(3,n:m2), 'r', 'LineWidth', LW)
hold off
grid on
title('Force z','Interpreter','latex','Fontsize', FS)
ylabel('Force [N]','Interpreter','latex','Fontsize', FS)
leg = legend('EE','collision');
set(leg, 'Location', 'eastoutside',  'Interpreter', 'latex','Fontsize', FS);
xlim([time(nn) time(mm)])
xlabel('Time [sec]','Interpreter','latex','Fontsize', FS)

%% saving variables in mat files

% script meant to find out:
% - n1/n2 : over which time interval offset FT sensor collision is computed
% - n3/n4 : over which time interval offset FT sensor EE is computed
% - timeShift : looking at start force both ft sensors and calculating diff
% - nn/mm : start and end time of time interval where collisions take pla

% (1) time to compute mean offset ft sensors over
% recordings 2-12
    % arm_push_home = 110,117,10,25; arm_push_forward = 1,4,2,6; 
    % arm_push_moving = 12,16,17,26; base_push = 6,15,0,12
% recordings 15-12
    % armCtrlJoint_push = 345,380,46,62; armCtrlJoint_push_move = 155,180,50,70; 
    % armCtrlJoint_push_load = 0,10,1,5
% recordings 21-12
    % armctrl_armload_push =  305,310,1,6; armctrl_baseload_push = 71,80,1,7; 
    % armctrl_baseload_push2kg = 58,62,22,38; armctrl_basepush = 140,148,22,32;
    % armctrl_grabbing_push = 440,448,5,15; armctrl_grabbing_push2 = 106,116,15,45;
    % armctrl_grabbing_push_move = 45,55,2,10; armctrl_push_andreea = 31,35,11,19;
% recordings 24-01
    % armctrljoint_basepush_armmove = 0.5,4,3.5,5; armctrljoint_grab = 46,50,122.5,127.5;
    % armctrljoint_maria = 16,20,4,8; ocs2base_trot = 46,50,15,156;
    % ocs2ee_basepush_armmove = 17,21,16,20 
n1sec = 46;
n2sec = 50;
n3sec = 122.5;
n4sec = 127.5;

% (2) allign time vectors FT sensor data and estimated wrench
% recordings 2-12
    % arm_push_home = 20.07; arm_push_forward = 39.623; 
    % arm_push_moving = 9.395; base_push = 11.265
% recordings 15-12
    % armCtrlJoint_push = -11.7026; armCtrlJoint_push_move = 92.0056; 
    % armCtrlJoint_push_load = -17.96241
% recordings 21-12
    % armctrl_armload_push = 3.5479; armctrl_baseload_push = -5.3350; 
    % armctrl_baseload_push2kg = 27.2198; armctrl_basepush = -15.14;
    % armctrl_grabbing_push = 0.9526; armctrl_grabbing_push2 = -1.9044;
    % armctrl_grabbing_push_move = -0.9359; armctrl_push_andreea = 8.8929;
% recordings 24-01
    % armctrljoint_basepush_armmove = 1.55653; armctrljoint_grab = 4.7758;
    % armctrljoint_maria = -0.12569; ocs2base_trot = 17.086;
    % ocs2ee_basepush_armmove = -0.1183;
timeShift = 4.7758;

% (3) part of interval where collisions take place
% first interval is with start and end calibration, second without
% recordings 2-12
    % arm_push_home = 26-155 | 35-148; arm_push_forward = 41-127 | 50-127; 
    % arm_push_moving = 13-118 | 20-110; base_push = 11-97 | 21-91
% recordings 15-12
    % armCtrlJoint_push = 33-514 | 64-502; armCtrlJoint_push_move = 110-302 | 123-283 (?); 
    % armCtrlJoint_push_load = 4-137 | 12-118
% recordings 21-12
    % armctrl_armload_push = 16-421 | 22-421; armctrl_baseload_push = 9-115 | 18-115; 
    % armctrl_baseload_push2kg = 41-175 | 45-175; armctrl_basepush = 33-180 | 51-174;
    % armctrl_grabbing_push = 16-616 | 29-616; armctrl_grabbing_push2 = 6-318 | 8-309;
    % armctrl_grabbing_push_move = 12-173 | 71-166 | 15-166 (including arm move); 
    % armctrl_push_andreea = 21-93 | 30-93;
% recordings 24-01
    % armctrljoint_basepush_armmove =  9-100 | 21-100; armctrljoint_grab = 36-97 | 49-97;
    % armctrljoint_maria = 8-89 | 18-89; ocs2base_trot = 8-151 | 8-143 (or 51-143 without trot);
    % ocs2ee_basepush_armmove = 12-108 | 24-108
nnsec = 49;
mmsec = 97;

% (4) save file
% recordings 2-12
    % filename = 'arm_push_home';
    % filename = 'arm_push_forward';
    % filename = 'arm_push_moving';
    % filename = 'base_push';
% recordings 15-12
    % filename='armCtrlJoint_push';
    % filename='armCtrlJoint_push_move';
    % filename='armCtrlJoint_push_load';
% recordings 21-12
    % filename = 'armctrl_armload_push';
    % filename = 'armctrl_baseload_push';
    % filename = 'armctrl_baseload_push2kg';
    % filename = 'armctrl_basepush';
    % filename = 'armctrl_grabbing_push';
    % filename = 'armctrl_grabbing_push2';
    % filename = 'armctrl_grabbing_push_move';
    % filename = 'armctrl_push_andreea';
% recordings 24-01
%     filename = 'armctrljoint_basepush_armmove';
%     filename = 'armctrljoint_grab';
%     filename = 'armctrljoint_maria';
%     filename = 'ocs2base_trot';
%     filename = 'ocs2ee_basepush_armmove';
save(filename,'n1sec','n2sec','n3sec','n4sec','timeShift','nnsec','mmsec');