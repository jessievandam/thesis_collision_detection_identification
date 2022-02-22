function [timesCollisionCutFinal,timesFTCutFinal, delay, delayAv] =...
                compute_detection_delay(time,timeftShifted,collision,...
                endInd,magFTForce, minTimeCol, minHeightCol, nnsec, mmsec, Ts)

%   Delay between start of real force and when it is detected in estimation

    % params
    K = size(collision,1);
    H = size(collision,2);
    
    %% compute start times detected collision

    % save times of detected collisions
    % initialize to endInd (arbitrary too large number)
    timesCollision = cell(K,H);
    count = cell(K,H);
    for k = 1:K % loop over all external torques
          for h = 1:H % loop over all jacobians
              timesCollision{k,h} = zeros(endInd,1);
              count{k,h} = 1;
          end
    end
    for k = 1:K % loop over all external torques
          for h = 1:H % loop over all jacobians
              for i = 2:endInd
                  if collision{k,h}(i) == 1 && collision{k,h}(i-1) == 0 
                      timesCollision{k,h}(count{k,h}) = time(i);
                      count{k,h} = count{k,h}+1;
                  end
              end
          end
    end   
    % remove non used array
    for k = 1:K % loop over all external torques
          for h = 1:H % loop over all jacobians
              timesCollision{k,h} = timesCollision{k,h}(1:count{k,h}-1);
          end
    end
    
    % only take times within the collision interval
    timesCollisionCut = cell(K,H);
    count = cell(K,H);
    for k = 1:K % loop over all external torques
          for h = 1:H % loop over all jacobians
              timesCollisionCut{k,h} = zeros(endInd,1);
              count{k,h} = 1;
          end
    end
    for k = 1:K % loop over all external torques
          for h = 1:H % loop over all jacobians
              for i = 1:size(timesCollision{k,h},1)
                  if timesCollision{k,h}(i)> nnsec &&  timesCollision{k,h}(i)< mmsec
                      timesCollisionCut{k,h}(count{k,h}) = timesCollision{k,h}(i);
                      count{k,h} = count{k,h}+1;
                  end
              end
          end
    end   
    % remove non used array
    for k = 1:K % loop over all external torques
          for h = 1:H % loop over all jacobians
              timesCollisionCut{k,h} = timesCollisionCut{k,h}(1:count{k,h}-1);
          end
    end
    
    % filter out times of peaks actually belonging to a collision   
    timesCollisionCutFinal = cell(K,H);
    count = cell(K,H);
    for k = 1:K % loop over all external torques
          for h = 1:H % loop over all jacobians
              timesCollisionCutFinal{k,h} = zeros(endInd,1);
              timesCollisionCutFinal{k,h}(1,1) = timesCollisionCut{k,h}(1,1);
              count{k,h} = 2;
          end
    end
    for k = 1:K % loop over all external torques
          for h = 1:H % loop over all jacobians
              for i = 2:size(timesCollisionCut{k,h},1)
                indexCollision = find(timeftShifted>timesCollisionCut{k,h}(i),1);
                % filter out end of collision detection and FNs
                if (timesCollisionCut{k,h}(i)-timesCollisionCutFinal{k,h}(count{k,h}-1))>minTimeCol  && ...   
                        mean(magFTForce((indexCollision-(1.2/Ts)):indexCollision-(0.4/Ts))) < minHeightCol && ... 
                        mean(magFTForce(indexCollision:(indexCollision+(1.5/Ts)))) > minHeightCol 
                    timesCollisionCutFinal{k,h}(count{k,h}) = timesCollisionCut{k,h}(i);
                    count{k,h} = count{k,h} + 1;
                end
               end
          end
    end   
    % remove non used array
    for k = 1:K % loop over all external torques
          for h = 1:H % loop over all jacobians
              timesCollisionCutFinal{k,h} = timesCollisionCutFinal{k,h}(1:count{k,h}-1);  
          end     
    end 
    
    %% compute start times collision F/T sensor ground truth force

    % check in magnitude of F/T sensor force when there is a sudden change in variance
    vectorChanges = ischange(magFTForce, 'variance', 'Threshold', 5000);
    indicesFT = find(vectorChanges == 1);
    timesFT = timeftShifted(indicesFT);

    % only take times within the collision interval
    count = 1;
    timesFTCut = zeros(endInd,1);
    for i = 1:size(timesFT,1)
        if timesFT(i)> nnsec &&  timesFT(i)< mmsec
            timesFTCut(count) = timesFT(i);
            count = count + 1;
        end
    end
    timesFTCut = timesFTCut(1:count-1);

    % filter out times of peaks within the collision by looking at min time between collisions
    % and min magnitude force between the peak and two sec before, s.t. not end of collision is detected
    count = 2;
    timesFTCutFinal = zeros(endInd,1);
    timesFTCutFinal(1,1) = timesFTCut(1,1);
    for i = 2:size(timesFTCut,1)
        indexCollision = find(timeftShifted>timesFTCut(i),1);
        if (timesFTCut(i)-timesFTCutFinal(count-1))>minTimeCol  && ...
                mean(magFTForce((indexCollision-(1/Ts)):indexCollision)) < minHeightCol
            timesFTCutFinal(count) = timesFTCut(i);
            count = count + 1;
        end
    end
    timesFTCutFinal = timesFTCutFinal(1:count-1);   
    
    %% compute delay
    
    delay = cell(K,H);
    delayAv = cell(K,H);
    for k = 1:K % loop over all external torques
          for h = 1:H % loop over all jacobians
              delaySize = min(size(timesFTCutFinal,1), size(timesCollisionCutFinal{k,h},1));
              delay{k,h} = timesCollisionCutFinal{k,h}(1:delaySize) - timesFTCutFinal(1:delaySize);
              delayAv{k,h} = mean(delay{k,h});      
          end
    end    
end

