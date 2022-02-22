function [torques] = momentum_observer_higher_order(taum, nonlinearTerms, massMatrix,...
                        qd, endInd, Ts, S_transposed, K_O1, K_O2, K_O3)

%   Momentum based observer higher-order

    % initialize
    torques = zeros(24,endInd);
    nonInertialTerms1 = zeros(24,1);
    nonInertialTerms2 = zeros(24,1);
    nonInertialTerms = zeros(24,1);
    
    for i = 2:endInd
        
        %% compute fe
        % compute non inertia terms
        newNonInertialTerms1 = S_transposed * taum(:,i) - nonlinearTerms(:,i); 
        % integrate
        nonInertialTerms1 = nonInertialTerms1 + newNonInertialTerms1 * Ts; 
        % estimated external torques
        fe = massMatrix{1,i} * qd(:,i) - nonInertialTerms1; 
        
        %% compute r(t)
        % integral 1       
        newNonInertialTerms = S_transposed * taum(:,i) - nonlinearTerms(:,i) + torques(:,i-1); 
        nonInertialTerms = nonInertialTerms + newNonInertialTerms * Ts; 
        int1 = -K_O2*torques(:,i-1) + massMatrix{1,i}*qd(:,i) - nonInertialTerms;
        
        % integral 2
        nonInertialTerms2 = nonInertialTerms2 + int1 * Ts;
        
        % total
        torques(:,i) = K_O1*K_O3*fe + K_O1*nonInertialTerms2;
        
    end

end