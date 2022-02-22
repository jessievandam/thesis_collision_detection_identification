function [collision] = detection_time(force,thresh,endInd,Ts, T_twopeaks, T_rippling,time)

%   Detect collision based on unfiltered x and y force and filtered z force

      % initialize
      collision = zeros(1,endInd);
      signCollision = 0;
      phaseCollision = 0;
      runOnce = 1;
      count = 0;
    
      % detect collision
      for i = 1:endInd   
          % there is no collision
          if phaseCollision == 0

              % collision detected
              if (abs(force(1,i))>thresh(1,i) || abs(force(2,i))>thresh(2,i) || ...
                      abs(force(3,i))>thresh(3,i) )

                  collision(i) = 1;
                  % indicates start of collision 
                  phaseCollision = 1;      

                  % check once on which axis and in which direction the collision is detected
                  if runOnce == 1
                      if force(1,i)>thresh(1,i)
                          signCollision = 1;
                          axis = 1;
                      elseif force(2,i)>thresh(2,i)
                          signCollision = 1;
                          axis = 2;
                      elseif force(3,i)>thresh(3,i)
                          signCollision = 1;
                          axis = 3;
                      elseif force(1,i)<-thresh(1,i)
                          signCollision = -1;
                          axis = 1;
                      elseif force(2,i)<-thresh(2,i)
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
                  collision(i) = 0;
              end

          % if collision has started but not ended yet
          elseif phaseCollision == 1

              % collision still lasting although not detected
              collision(i) = 1; 

              % if collision was detected positive, wait until it is detected negative
              if signCollision == 1 
                  if (axis == 1 && force(1,i)<-thresh(1,i)) ||...
                          (axis == 2 && force(2,i)<-thresh(2,i)) ||...
                          (axis == 3 && force(3,i)<-thresh(3,i))
                      collision(i) = 0;
                      phaseCollision = 2; % after this, collision has ended
                      count = 0; % make sure phase 2 starts with 0 count
                  end

              % if collision was detected negative, wait until it is detected positive
              elseif signCollision == -1 
                  if (axis == 1 && force(1,i)>thresh(1,i)) ||...
                          (axis == 2 && force(2,i)>thresh(2,i)) ||...
                          (axis == 3 && force(3,i)>thresh(3,i))
                      collision(i) = 0;
                      phaseCollision = 2; % after this, collision has ended
                      count = 0; % make sure phase 2 starts with 0 count
                  end
              end   

              % in case the second peak doesn't appear for T_twopeaks second after the collision has ended
              % go straight to phase 0 again and skip phase 2
              if abs(force(1,i))<thresh(1,i) && abs(force(2,i))<thresh(2,i) && ...
                  abs(force(3,i))<thresh(3,i) 

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

               collision(i) = 0;   

               % detect when all peaks are below threshold for at least 0.1 sec
               if abs(force(1,i))<thresh(1,i) && abs(force(2,i))<thresh(2,i) && ...
                  abs(force(3,i))<thresh(3,i) 

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

