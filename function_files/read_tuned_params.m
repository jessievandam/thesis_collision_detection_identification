function [K_O, FC, N, Af, Qp, Qf, K_O1, K_O2, K_O3] = read_tuned_params(Ts)

%   Tuned Kalman, MBO and GM observer filter matrices

    % MBO: gain matrix [s^-1]
    gains = [15, 15, 3, 15, 3, 15,... % base
            40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,... % legs
            10, 10, 10, 10, 10, 10];  % arm
    K_O = diag(gains);  
    
    % MBO higher-order: three gain matrices [s^-1]
    gains1 = 12*ones(24,1);
    K_O1 = diag(gains1);  
    gains2 = 0.8*ones(24,1);
    K_O2 = diag(gains2);  
    gains3 = 0.8*ones(24,1);
    K_O3 = diag(gains3);

    % GM: cut-off frequency [Hz]
    FC = exp(-Ts*gains);

    % Kalman
    N = 24;
    Af = zeros(N,N);   % determines dynamics external wrench: set to negative diagonal to mitigate offset
    Qp = eye(N,N);     % process noise in joint friction, the larger the weight, the more friction uncertainty
    Qf = 500*eye(N,N); % process noise in contact forces and torques (example: 4x higher than Qp). the larger the weights,
                       % the less kalman filter will rely on fdot = 0 (contact forces being constant) thus faster response,
                       % however, also the larger noise amplification 

    % larger weight = smoother signal, but more delay (and additional ripples might appear)
    % base
    Qp(1:6,1:6) = Qp(1:6,1:6)*10; % set higher for base collisions, 800 original
    % legs: second joint shows more friction uncertainty
    Qp(8,8) = Qp(8,8);
    Qp(11,11) = Qp(11,11);
    Qp(14,14) = Qp(14,14);
    Qp(17,17) = Qp(17,17);
    % arm: more friction uncertainty than legs and base
    Qp(19:24,19:24) = Qp(19:24,19:24)*20; 

    % base and arm: faster response required
    Qf(1:6,1:6) = Qf(1:6,1:6)*10;
    Qf(19:24,19:24) = Qf(19:24,19:24)*10; 

end

