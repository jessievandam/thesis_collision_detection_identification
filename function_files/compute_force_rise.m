function [forceRise, forceRiseAv] = compute_force_rise(timesCollisionCutFinal,timesFTCutFinal,...
                        magFTForce, time, timeftShifted)

%    Compute the percentage of rise during the detection delay

    %% indices of peaks
    
    % end indices of FT and collision force
    n = find(time>0,1);
    m1 = find(time>(min([time(end), timeftShifted(end)])-1),1);          % estimated force endInd
    m2 = find(timeftShifted>(min([time(end), timeftShifted(end)])-1),1); % ft sensor collision endInd

    % time intervals in which the FT sensor peaks occur
    nCol = size(timesFTCutFinal,1);
    timeIntervals(1:nCol) = timesFTCutFinal;
    timeIntervals(nCol+1) = time(m1-5); % to mark the end of last interval, -5 because otherwise the end can't be found    

    % indices of time vector of time intervals
    x = zeros(nCol+1,1);   % index of time interval time vector
    xx = zeros(nCol+1,1);  % index of time interval peaks vector
    for i = 1:(nCol+1)
        x(i) = find(timeftShifted>timeIntervals(i),1); 
    end
    
    % indices of peaks vector of time intervals
    [FTallpeaks, locsFT] = findpeaks(magFTForce(1,n:m2));
    FTpeaks = zeros(nCol,1);
    for i = 1:(nCol+1)
        if i ~= nCol+1
            xx(i) = find(locsFT>x(i),1);
        else
            xx(i) = size(locsFT,2);
        end
        if i>1
            FTpeaks(i-1) = max(FTallpeaks(1,xx(i-1):xx(i)));
        end
    end

    %% compute force rise
    
    forceRise = zeros(nCol,1);
    for i = 1:nCol
        
        % if a small offset is present in FT force magnitude at start
        % force, this is subtracted
        indStartForce = find(timeftShifted>timesFTCutFinal(i),1)-50; 
        % FT force magnitude when collision is detected
        indDetectionForce = find(timeftShifted>timesCollisionCutFinal(i),1);
       
        forceRise(i,1) = (magFTForce(indDetectionForce) - magFTForce(indStartForce))/(FTpeaks(i) - magFTForce(indStartForce))*100;
    end
    
    % average
    forceRiseAv = mean(forceRise);
    
end

