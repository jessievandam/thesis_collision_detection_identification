function [torques] = motor_error(taum, nonlinearTerms, endInd, S_transposed)

%   Estimation based on error in motor torques

    %% computing external joint torques
    torques = zeros(24,endInd);
    for i = 2:endInd
        torques(:,i) = -S_transposed*taum(:,i) + nonlinearTerms(:,i);
    end

end