function [torques] = direct_estimation(taum, nonlinearTerms, massMatrix, qdd, endInd, S_transposed, timeVec)

%   Direct estimation

    %% add LPF to acceleration measurements: not used to prevent delay
    
    % LPF transfer function
    fc = 2.0; % cut-off freq
    sysLPF = tf([0.0, fc*2*pi], [1.0, fc*2*pi]);
    qddLPF = zeros( size(qdd,1), size(qdd,2) );
    
    for i = 1:24
        qddLPF(i,:) = lsim(sysLPF,qdd(i,:),timeVec(1:size(qdd,2)));
    end

    %% computing external joint torques
    torques = zeros(24,endInd);
    for i = 2:endInd % starting at two because of differentiation qdd
        torques(:,i) = -S_transposed*taum(:,i) + nonlinearTerms(:,i) + massMatrix{1,i}*qdd(:,i-1);
    end

end

