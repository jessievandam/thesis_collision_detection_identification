function [forceBPF, forceHPF] = filtering_time_domain(cutOffFreqMin, cutOffFreqMax, force, timeVec, endInd)

%   Time domain filtering

    %% add BPF to force, input from Hz to rad/sec
    sysBPF = tf([0.0, cutOffFreqMax*2*pi, 0.0], [1.0, (cutOffFreqMax*2*pi+cutOffFreqMin*2*pi), (cutOffFreqMax*cutOffFreqMin*4*pi*pi)]);
    forceBPF = cell(2,1);
    forceBPF{1,1} = zeros(3, endInd);
    forceBPF{2,1} = zeros(3, endInd);
    for j = 1:2
        for i = 1:3
            forceBPF{j,1}(i,:) = lsim(sysBPF,force{j,1}(i,:),timeVec);
        end
    end
    
    
    %% add HPF to force, input from Hz to rad/sec
    sysHPF = tf([1.0, 0.0], [1.0, cutOffFreqMin*2*pi]);
    forceHPF = cell(2,1);
    forceHPF{1,1} = zeros(3, endInd);
    forceHPF{2,1} = zeros(3, endInd);
    for j = 1:2
        for i = 1:3
                forceHPF{j,1}(i,:) = lsim(sysHPF,force{j,1}(i,:),timeVec);
        end
    end
    
end

