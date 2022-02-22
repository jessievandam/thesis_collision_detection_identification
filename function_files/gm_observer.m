function [torques] = gm_observer(taum, nonlinearTerms, massMatrix, qd, endInd, Ts, S_transposed, FC)

%   Discrete-time momentum-based observer

    % gains gm observer
    gamma = zeros(24,24);
    beta = zeros(24,24);
    for i = 1:24
        gamma(i,i) = FC(i); 
        beta(i,i) = (1-gamma(i,i)) / (gamma(i,i)*Ts);
    end
    
    % initialize
    torques = zeros(24,endInd);
    filterState = zeros(24,1);
    massMatrixPrev = zeros(24,24);
    
    for i = 2:endInd
        massMatrixDeriv = (massMatrix{1,i} - massMatrixPrev)/Ts;
        massMatrixPrev = massMatrix{1,i};

        betaMomentum = beta*(massMatrix{1,i} * qd(:,i));
        filterInput = betaMomentum + S_transposed * taum(:,i) - nonlinearTerms(:,i);
        filterInput = filterInput + massMatrixDeriv*qd(:,i);
        filterState = gamma*filterState + (eye(24,24)-gamma)*filterInput;
        torques(:,i) = betaMomentum - filterState;
    end

end