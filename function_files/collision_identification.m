function [magEstForceFinal, magEstForceLPFfinal, offset] = collision_identification(collision,...
                magEstForce, magEstForceLPF, endInd, Ts)

%   Subtraction of offset
 
    % params
    K = size(magEstForce,1);
    H = size(magEstForce,2);
    T_fixed = 0.2;
    
    %% apply LPF to estimated force: offset
    
    fc = 0.5; % cut-off frequency [Hz]
    alpha = exp(-fc*2*pi*Ts);
    offset = cell(K,H);
    % intialize 
    runOnce = 1;
    count = ceil(T_fixed/Ts);
    offsetFixed = 0;
    for k = 1:K
      for h = 1:H 
          offset{k,h} = zeros(1,endInd);
      end
    end
    % low-pass filter
    for k = 1:K
        for h = 1:H 
            for i = 2:endInd
                if collision{k,h}(i) == 0
                    % if collision just ended 
                    if collision{k,h}(i-1) == 1 
                        offset{k,h}(i) = offsetFixed;
                        % start counting
                        count = 0; 
                    elseif count < ceil(T_fixed/Ts)
                       % keep offset fixed for T_fixed sec
                       offset{k,h}(i) = offsetFixed;
                       count = count + 1;
                    else
                       offset{k,h}(i) = alpha*offset{k,h}(i-1) + (1-alpha)*magEstForce{k,h}(i);
                    end
                    runOnce = 1;
%                     offset{k,h}(i) = alpha*offset{k,h}(i-1) + (1-alpha)*magEstForce{k,h}(i);
                else
                    % freeze offset
                    if runOnce == 1 
                        offsetFixed = offset{k,h}(i-1);
                        runOnce = 0; 
                    end
                    offset{k,h}(i) = offsetFixed;
                end
            end
        end
    end
    
    %% subtract offset
    
    % initialize
    magEstForceFinal = cell(K,H);
    magEstForceLPFfinal = cell(K,H);
    for k = 1:K % loop over all external torques
         for h = 1:H % loop over all jacobians
             magEstForceFinal{k,h} = zeros(1,endInd);
             magEstForceLPFfinal{k,h} = zeros(1,endInd);
         end
    end
   
    for k = 1:K % loop over all external torques
         for h = 1:H % loop over all jacobians    
              for i = 2:endInd  
                  % constantly subtract offset
                  magEstForceFinal{k,h}(1,i) =  magEstForce{k,h}(1,i) -  offset{k,h}(i);
                  magEstForceLPFfinal{k,h}(1,i) =  magEstForceLPF{k,h}(1,i) -  offset{k,h}(i);
              end
         end
    end
    
end

