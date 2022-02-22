function [collision] = detection_freq_hann(force,thresh,endInd,Ts, T_rippling)

%   Detect collision based on unfiltered x and y force and filtered z force

      % initialize
      collision = zeros(1,endInd);
      runOnce = 1;
      phaseCollision = 0;

      % detect collision
      for i = 1:endInd   
          
          % there is no collision
          if phaseCollision == 0
              
              % collision detected
              if force(1,i)>thresh(1,i) || force(2,i)>thresh(2,i) || ...
                      force(3,i)>thresh(3,i) 
                  collision(i) = 1;
                  phaseCollision = 1;
                  count = 0;
                  
                  % check once on which axis and in which direction the collision is detected
                  if runOnce == 1
                      if force(1,i)>thresh(1,i)
                          axis = 1;
                      elseif force(2,i)>thresh(2,i)
                          axis = 2;
                      else
                          axis = 3;
                      end
                      runOnce = 0;
                  end
              end
          
          % detect end of collision 
          elseif phaseCollision == 1
              collision(i) = 1;  
              
              % check when detected collision peak goes below threshold
              if (axis == 1 && force(1,i)<thresh(1,i)) ||...
                      (axis == 2 && force(2,i)<thresh(2,i)) ||...
                      (axis == 3 && force(3,i)<thresh(3,i))
                  collision(i) = 1;
                  phaseCollision = 2;
              end
          
          % if end of collision has been detected, but the force is still showing rippling
          elseif phaseCollision == 2

               collision(i) = 0;   

               % detect when all peaks are below threshold for at least T_rippling
               % sec,to prevent discontinuous collision detection during rippling
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

