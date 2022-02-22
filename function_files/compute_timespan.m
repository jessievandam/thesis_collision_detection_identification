function [timesCol,timesEstimation, error, accuracy, timesFTCutFinalStop] = compute_timespan(timesFTCutFinal,time,...
                    timesCollisionCutFinal, collision, jacobian, endInd, magFTForce, timeftShifted)

%   Compute time span of ground truth and estimated collision forces

    % Params
    nCol = size(timesFTCutFinal,1);
    % choose jacobian for which to compare wrench  
    j1 = jacobian;  
  
    % Ground truth collision force
    timesFTCutFinalStop= zeros(nCol,1);
    count = 1;  
        for i = 1:endInd
            indFT = find(timeftShifted>time(i),1);
            if count <= nCol && time(i)>(timesFTCutFinal(count)+0.2) && ...
                    magFTForce(indFT) < 2      
                timesFTCutFinalStop(count,1) = time(i);
                count = count + 1;
            end
        end
    timesCol = timesFTCutFinalStop-timesFTCutFinal;

    % Estimated force
    timesCollisionCutFinalStop = zeros(nCol,1);
    count = 1;   
    for i = 1:endInd
        if count <= nCol && time(i)>timesCollisionCutFinal{4,j1}(count) && ...
                collision{4,j1}(i-1) == 1 && collision{4,j1}(i) == 0          
            timesCollisionCutFinalStop(count,1) = time(i);
            count = count + 1;
        end 
    end
    timesEstimation = timesCollisionCutFinalStop(:,1) - timesCollisionCutFinal{4,j1}(:,1);
    
    % Error
    error = (1-(timesEstimation./timesCol))*100;
    accuracy = (timesEstimation./timesCol)*100;
end

