function [torques] = momentum_observer(taum, nonlinearTerms, massMatrix, qd, endInd, Ts, S_transposed, K_O)

%   Momentum based observer

    % initialize
    torques = zeros(24,endInd);
    nonInertialTerms = zeros(24,1);

    for i = 2:endInd
        % compute non inertia terms
        newNonInertialTerms = S_transposed * taum(:,i) - nonlinearTerms(:,i) + torques(:,i-1); 
        % integrate
        nonInertialTerms = nonInertialTerms + newNonInertialTerms * Ts;
        % estimated external torques
        torques(:,i) = K_O * (massMatrix{1,i} * qd(:,i) - nonInertialTerms); 
    end

end