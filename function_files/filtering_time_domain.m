function [forceBPF, forceHPF] = filtering_time_domain(cutOffFreqMin,cutOffFreqMax,force, timeVec, endInd)

%   Time domain filtering

    %% add BPF to force, input from Hz to rad/sec
    sysBPF = tf([0.0, cutOffFreqMax*2*pi, 0.0], [1.0, (cutOffFreqMax*2*pi+cutOffFreqMin*2*pi), (cutOffFreqMax*cutOffFreqMin*4*pi*pi)]);
    forceBPF = zeros(3, endInd);
    for i = 1:3
        forceBPF(i,:) = lsim(sysBPF,force(i,:),timeVec);
    end
    
    
    %% add HPF to force, input from Hz to rad/sec
    sysHPF = tf([1.0, 0.0], [1.0, cutOffFreqMin*2*pi]);
    forceHPF = zeros(3, endInd);
    for i = 1:3
            forceHPF(i,:) = lsim(sysHPF,force(i,:),timeVec);
    end
    
end

