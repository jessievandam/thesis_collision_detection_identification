function [torques] = kalman_observer(taum, nonlinearTerms, massMatrix, qd, endInd,...
                                      Ts, S_transposed, Af, Qp, Qf, N)

%   Kalman based momentum observer

    %% initialization
    
    % measurement matrix C
    C = [eye(N), zeros(N,N)];

    % velocity covariance matrix Qqd during arm movement
    load('Qqd.mat'); 

    % total covariance matrix
    Qc = [Qp, zeros(N,N);    
          zeros(N,N), Qf]; % the larger, the faster the response to changes in f; common weights are same responsiveness
                           % base and legs weights smaller since we want less responsiveness to base changes ?  

    % initialize for update step
    Bc = [eye(N,N); zeros(N,N)];
    Pprev = eye(2*N);
    xprev = zeros(2*N,1);
    torques = zeros(N,endInd);
    identityTorques = eye(N,N);
    
    %% update step

    for i = 1:endInd

        % compute time varying Rd
        Rc = massMatrix{1,i}*Qqd*massMatrix{1,i}';
        Rd = 1/Ts * Rc;

        % continuous matrices
        Ac = [zeros(N,N), identityTorques;
              zeros(N,N), Af];

        % discretize Ac, Bc
        expAcBc = expm([Ac, Bc; zeros(N,N*2), zeros(N,N)]*Ts);
        Ak = expAcBc(1:(N*2), 1:(N*2));
        Bk = expAcBc(1:(N*2), (N*2+1):end);

        % obtain discretized covariance matrix
        H = [Ac, Qc;
            zeros(N*2,N*2), -Ac'];
        M = expm(H*Ts);
        M11 = M(1:(N*2), 1:(N*2));
        M12 = M(1:(N*2), (N*2+1):end);
        Qk = M12.* M11';

        % input u
        u = S_transposed * taum(:,i) - nonlinearTerms(:,i);

        % output vector y
        y = massMatrix{1,i}*qd(:,i);

        % predict state and covariance matrices
        x = Ak*xprev + Bk*u;
        Pk = Ak*Pprev*Ak' + Qk;

        % calculate kalman gain
        K = Pk*C' * inv(C*Pk*C' + Rd);

        % correct state and covariance matrices estimates with measurement update
        x = x + K*(y - C*x);
        Pk = (eye(N*2) - K*C) * Pk * (eye(N*2) - K*C).' + K*Rd*K';

        % save previous values
        Pprev = Pk;
        xprev = x; 

        % extract contact force estimates from state vector x
        torques(:,i) = [zeros(N,N), eye(N)] * x;
    end

end