%% open rosbags
bag1 = rosbag('run_all_methods.bag');
bag2 = rosbag('run_all_methods2.bag');
bag3 = rosbag('run_all_methods3.bag');

%% read out computation times

% computation times 1
compTimes1_bag = select(bag1,'Topic','/ocs2_controller_collision/computation_times');
compTimes1_struct = readMessages(compTimes1_bag,'DataFormat','struct');

% computation times 2
compTimes2_bag = select(bag2,'Topic','/ocs2_controller_collision/computation_times');
compTimes2_struct = readMessages(compTimes2_bag,'DataFormat','struct');

% computation times 3
compTimes3_bag = select(bag3,'Topic','/ocs2_controller_collision/computation_times');
compTimes3_struct = readMessages(compTimes3_bag,'DataFormat','struct');

% time
time1 = compTimes1_bag.MessageList.Time;
time1 = time1 - time1(1,1);
endInd1 = size(time1,1);

time2 = compTimes2_bag.MessageList.Time;
time2 = time2 - time2(1,1);
endInd2 = size(time2,1);

time3 = compTimes3_bag.MessageList.Time;
time3 = time3 - time3(1,1);
endInd3 = size(time1,1);

% save in vectors over time
compTimes1 = zeros(13,endInd1);
compTimes2 = zeros(13,endInd2);
compTimes3 = zeros(13,endInd3);
for i = 1:endInd1
  compTimes1(:,i) = compTimes1_struct{i,1}.Data(:,1);
end
for i = 1:endInd2
  compTimes2(:,i) = compTimes2_struct{i,1}.Data(:,1);
end
for i = 1:endInd3
  compTimes3(:,i) = compTimes3_struct{i,1}.Data(:,1);
end

%% compute averages

avCompTimes1 = mean(compTimes1,2);
avCompTimes2 = mean(compTimes2,2);
avCompTimes3 = mean(compTimes3,2);

avCompTimes = (avCompTimes1+avCompTimes2+avCompTimes3)/3;
totalTime = time1(end) + time2(end) + time3(end);

% average computation times [ms] taken over total time 578 sec
% order computation times:
% 1 MBO, 2 GM, 3 direct, 4 kalman, 5 filter time domain, 6 filter FFT, 
% 7 filter DFT, 8 const thresh, 9 dyn thresh vel, 10 dyn thresh std,
% 11 wrench estimation, 12 offset and ft sensor subtraction, 
% 13 reading alma c and state values

method = ["MBO";"GM";"Direct estimation";"Kalman";"Filter time domain";"Filter freq domain FFT";
           "Filter freq domain DFT"; "Const thresh"; "Dyn thresh vel"; "Dyn thresh STD";
           "Wrench estimation"; "Offset and FT force subtraction"; "Reading AlmaC variables"];
T = table(method, avCompTimes);
% writetable(T,"computation_times_table",'Delimiter','tab');


