function [collision, forceBPF, forceHPF] = collision_detection(force, timeVec,...
                        endInd, nnsec, mmsec, time, Ts, T_twopeaks, T_rippling, ...
                        cutOffFreqMin, cutOffFreqMax, constThresh)

%   Simple collision detection for arm only with BPF and constant threshold

    % params
    K = size(force,1);
    H = size(force,2);
    
    %% add BPF to force, input from Hz to rad/sec
    sysBPF = tf([0.0, cutOffFreqMax*2*pi, 0.0], [1.0, (cutOffFreqMax*2*pi+cutOffFreqMin*2*pi), (cutOffFreqMax*cutOffFreqMin*4*pi*pi)]);
    forceBPF = cell(K,H);
    for k = 1:K % loop over all external torques
          for h = 1:H % loop over all jacobians
              for l = 1:3
                  forceBPF{k,h}(l,:) = lsim(sysBPF,force{k,h}(l,:),timeVec);
              end   
          end
    end
    
    %% add HPF to force, input from Hz to rad/sec
    sysHPF = tf([1.0, 0.0], [1.0, cutOffFreqMin*2*pi]);
    forceHPF = cell(K,H);
    for k = 1:K % loop over all external torques
          for h = 1:H % loop over all jacobians
              for l = 1:3
                  forceHPF{k,h}(l,:) = lsim(sysHPF,force{k,h}(l,:),timeVec);
              end   
          end
    end
 
    %% detect collision
  
    % detection force: HPF or BPF
    forceDet = forceBPF;

    % initialize
    collision = cell(K,H);
    for k = 1:K % loop over all external torques
          for h = 1:H % loop over all jacobians
              collision{k,h} = zeros(1,endInd);
          end
    end
    
    % only time interval where collisions take place
    nn = find(time>nnsec,1); 
    mm = find(time>mmsec,1);    
    
    %detect collision
    for k = 1:K % loop over all external torques
         for h = 1:H % loop over all jacobians         
             
              % intialization
              signCollision = 0;
              phaseCollision = 0;
              runOnce = 1;
              count = 0;
    
              for i = nn:mm
                  
                  % there is no collision
                  if phaseCollision == 0
                      
                      % collision detected
                      if (abs(forceDet{k,h}(1,i))>constThresh(1) || abs(forceDet{k,h}(2,i))>constThresh(2) || ...
                              abs(forceDet{k,h}(3,i))>constThresh(3) )
                          
                          collision{k,h}(i) = 1;
                          % indicates start of collision 
                          phaseCollision = 1;      

                          % check once on which axis and in which direction the collision is detected
                          if runOnce == 1
                              if forceDet{k,h}(1,i)>constThresh(1)
                                  signCollision = 1;
                                  axis = 1;
                              elseif forceDet{k,h}(2,i)>constThresh(2)
                                  signCollision = 1;
                                  axis = 2;
                              elseif forceDet{k,h}(3,i)>constThresh(3)
                                  signCollision = 1;
                                  axis = 3;
                              elseif forceDet{k,h}(1,i)<-constThresh(1)
                                  signCollision = -1;
                                  axis = 1;
                              elseif forceDet{k,h}(2,i)<-constThresh(2)
                                  signCollision = -1;
                                  axis = 2;
                              else
                                  signCollision = -1;
                                  axis = 3;
                              end
                              runOnce = 0;
                          end
                      
                      % no collision
                      else
                          collision{k,h}(i) = 0;
                      end
                      
                  % if collision has started but not ended yet
                  elseif phaseCollision == 1
                      
                      % collision still lasting although not detected
                      collision{k,h}(i) = 1; 
                      
                      % if collision was detected positive, wait until it is detected negative
                      if signCollision == 1 
                          if (axis == 1 && forceDet{k,h}(1,i)<-constThresh(1)) ||...
                                  (axis == 2 && forceDet{k,h}(2,i)<-constThresh(2)) ||...
                                  (axis == 3 && forceDet{k,h}(3,i)<-constThresh(3))
                              collision{k,h}(i) = 0;
                              phaseCollision = 2; % after this, collision has ended
                              count = 0; % make sure phase 2 starts with 0 count
                          end
                          
                      % if collision was detected negative, wait until it is detected positive
                      elseif signCollision == -1 
                          if (axis == 1 && forceDet{k,h}(1,i)>constThresh(1)) ||...
                                  (axis == 2 && forceDet{k,h}(2,i)>constThresh(2)) ||...
                                  (axis == 3 && forceDet{k,h}(3,i)>constThresh(3))
                              collision{k,h}(i) = 0;
                              phaseCollision = 2; % after this, collision has ended
                              count = 0; % make sure phase 2 starts with 0 count
                          end
                      end   
                      
                      % in case the second peak doesn't appear for T_twopeaks full second after the collision has ended
                      % go straight to phase 0 again and skip phase 2
                      if abs(forceDet{k,h}(1,i))<constThresh(1) && abs(forceDet{k,h}(2,i))<constThresh(2) && ...
                          abs(forceDet{k,h}(3,i))<constThresh(3) 
                            
                            % count how long the peaks are below the threshold
                            count = count + 1;
                            if count > (T_twopeaks/Ts)
                                % reset variable and back to phase 0
                                runOnce = 1;
                                phaseCollision = 0;
                                count = 0;
                            end 
                       else
                           count = 0;
                       end
                  
                  % if end of collision has been detected, but the force is still crossing the threshold
                  elseif phaseCollision == 2

                       collision{k,h}(i) = 0;   

                       % detect when all peaks are below threshold for at
                       % least T_rippling sec, to prevent additional detected
                       % collisions during rippling
                       if abs(forceDet{k,h}(1,i))<constThresh(1) && abs(forceDet{k,h}(2,i))<constThresh(2) && ...
                          abs(forceDet{k,h}(3,i))<constThresh(3) 
                            
                            % count how long the peaks are below the threshold
                            count = count + 1;
                            if count > (T_rippling/Ts)
                                % reset variable and back to phase 0
                                runOnce = 1;
                                phaseCollision = 0;
                                count = 0;
                            end
                       else
                           count = 0;
                       end
                  end  

              end
         end
    end

end

