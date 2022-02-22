function [timesCollisionCutFinal,timesFTCutFinal, delay, delayAv] =...
                compute_detection_delay_detection(time,timeftShifted,collision,...
                endInd,magFTForce, minTimeCol, minHeightCol, nnsec,...
                mmsec, Ts)

%   Delay between start of real force and when it is detected in estimation

    %% compute start times detected collision

    % save times of detected collisions
    % initialize to endInd (arbitrary too large number)
    timesCollision = zeros(endInd,1);
    count = 1;
    for i = 2:endInd
        if collision(i) == 1 && collision(i-1) == 0 
            timesCollision(count) = time(i);
            count = count+1;
        end
    end
    % remove non used array
    timesCollision = timesCollision(1:count-1);
    
    % only take times within the collision interval
    timesCollisionCut = zeros(endInd,1);
    count = 1;
    for i = 1:size(timesCollision,1)
        if timesCollision(i)> nnsec &&  timesCollision(i)< mmsec
            timesCollisionCut(count) = timesCollision(i);
            count = count+1;
        end
    end

    % remove non used array
    timesCollisionCut = timesCollisionCut(1:count-1);
  
    
    % filter out times of peaks actually belonging to a collision   
    timesCollisionCutFinal = zeros(endInd,1);
    timesCollisionCutFinal(1,1) = timesCollisionCut(1,1);
    count = 2;
    for i = 2:size(timesCollisionCut,1)
      indexCollision = find(timeftShifted>timesCollisionCut(i),1);
      % filter out end of collision detection and FNs
      if (timesCollisionCut(i)-timesCollisionCutFinal(count-1))>minTimeCol  && ...   
              mean(magFTForce((indexCollision-(1.5/Ts)):indexCollision-(0.7/Ts))) < minHeightCol && ... 
              mean(magFTForce(indexCollision:(indexCollision+(0.3/Ts)))) > minHeightCol 
          timesCollisionCutFinal(count) = timesCollisionCut(i);
          count = count + 1;
      end
    end   
    % remove non used array
    timesCollisionCutFinal = timesCollisionCutFinal(1:count-1); 
    
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
    
    delaySize = min(size(timesFTCutFinal,1), size(timesCollisionCutFinal,1));
    delay = timesCollisionCutFinal(1:delaySize) - timesFTCutFinal(1:delaySize);
    delayAv = mean(delay);      
end