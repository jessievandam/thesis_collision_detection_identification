function [overshoot, overshootAv, overEstimation, overEstimationAv, underEstimation, underEstimationAv,...
                   RMS, STD, estForcePeaksMin, estForcePeaksMinAv, FTpeaks, estForcePeaks] = ...
                   metrics_comparison(magEstForceFinal, magFTForce, magEstForce, time,...
                   timeftShifted, n, m1, m2, nnsec,...
                   timesFTCutFinal, jacobian)

%    Compare different external observers in wrench estimation 

%% determine time intervals where one collision takes place to later on determine peak of collision
    
    nCol = size(timesFTCutFinal,1);
    timeIntervals(1:nCol) = timesFTCutFinal;
    timeIntervals(nCol+1) = time(m1-5); % to mark the end of last interval, -5 because otherwise the end can't be found    

%% data set specific values
    
    % choose jacobian for which to compare wrench  
    j1 = jacobian;  
    % compare all K external observers
    K = size(magEstForceFinal,1);

%% [1] compute overshoot of estimated force in % of FT sensor force

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
    % if a small offset is present in FT force magnitude at start force, this is subtracted
    for i = 1:nCol
        indStartForce = find(timeftShifted>timesFTCutFinal(i),1)-100; 
        FTpeaks(i) = FTpeaks(i) - magFTForce(indStartForce); 
    end 
    
    % indices of time vector of time intervals
    y = zeros(nCol+1,1);  % index of time interval time vector
    yy = zeros(nCol+1,1); % index of time interval peaks vector
    for i = 1:(nCol+1)
        y(i) = find(time>timeIntervals(i),1); 
    end
    % indices of peaks vector of time intervals
    estForcePeaks = cell(K,1);
    estForcePeaksIndex = cell(K,1);
    for k = 1:K
        [estForceAllpeaks, locsEstForce] = findpeaks(magEstForceFinal{k,j1}(1,n:m1));

        estForcePeaks{k,1} = zeros(nCol,1);      % initialization
        estForcePeaksIndex{k,1} = zeros(nCol,1); % initialization
        for i = 1:(nCol+1)
            if i ~= nCol+1
                yy(i) = find(locsEstForce>y(i),1);
            else
                yy(i) = size(locsEstForce,2);
            end
            if i>1
                estForcePeaks{k,1}(i-1) = max(estForceAllpeaks(1,yy(i-1):yy(i)));
            end
        end
    end

    % final relative error
    overshoot = cell(K,1);
    overshootAv = zeros(K,1);
    for k = 1:K
        overshoot{k,1} = ((estForcePeaks{k,1}./FTpeaks)-1)*100;
        overshootAv(k,1) = mean(overshoot{k,1});
    end
    
    % compute average over- and underestimation
    overEstimation = cell(K,1);
    underEstimation = cell(K,1);
    overEstimationAv = zeros(K,1);
    underEstimationAv = zeros(K,1);
    for k = 1:K
        o = 1;
        u = 1;
        for i = 1:nCol 
            if overshoot{k,1}(i) > 0
                overEstimation{k,1}(o) = overshoot{k,1}(i);
                o = o + 1;
            else
                underEstimation{k,1}(u) = overshoot{k,1}(i);
                u = u + 1;
            end
        end
        overEstimationAv(k,1) = mean(overEstimation{k,1});
        underEstimationAv(k,1) = mean(underEstimation{k,1});
    end

%% [2] RMS (same as rmse) in standstill

    RMS = zeros(K,1);
    STD = zeros(K,1);
    startRMS = find(time>nnsec-1,1); 
    endRMS = find(time>timesFTCutFinal(1,1),1)-100;
    for k = 1:K
        if k == 1
            % RMS does not take offset into account: square the data points
            RMS(k,1) = rms(magEstForce{2,j1}(1,startRMS:endRMS));
            % std does take offset into account: square the difference between data point and mean
            STD(k,1) = std(magEstForce{2,j1}(1,startRMS:endRMS),0,2); 
        else
            % RMS does not take offset into account: square the data points
            RMS(k,1) = rms(magEstForceFinal{k,j1}(1,startRMS:endRMS));
            % std does take offset into account: square the difference between data point and mean
            STD(k,1) = std(magEstForce{k,j1}(1,startRMS:endRMS),0,2); 
        end
    end

%% [3] delay of external observers vs each other        
    
   % look at script compute_detection_delay
    
%% [4] downwards peak after collision ends

    % indices of peaks vector of time intervals
    estForcePeaksMin = cell(K,1);
    estForcePeaksMinAv = zeros(K,1);
    for k = 1:K
        % mirror signal to find minima
        [estForceAllpeaks, locsEstForce] = findpeaks(-magEstForceFinal{k,j1}(1,n:m1));
        % initialization
        estForcePeaksMin{k,1} = zeros(nCol,1); 
        for i = 1:(nCol+1)
            if i ~= nCol+1
                yy(i) = find(locsEstForce>y(i),1);
            else
                yy(i) = size(locsEstForce,2);
            end
            if i>1
                % mirror back
                estForcePeaksMin{k,1}(i-1) = -max(estForceAllpeaks(1,yy(i-1):yy(i)));
            end
        end
        % average downward peak error
        estForcePeaksMinAv(k,1) = sum(estForcePeaksMin{k,1}, 'all')/nCol;
    end

end

