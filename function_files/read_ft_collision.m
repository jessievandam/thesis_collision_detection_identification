function [timeftShifted, ftForceCol, magFTForce] = read_ft_collision(bagft,n1sec,n2sec,timeShift)

%   Read parameters FT sensor collision

    %% read out FT sensor values collision

    % collision ground truth wrench
    ftCollision_bag = select(bagft,'Topic','/rokubimini_cosmo/ft_sensor/wrench');
    ftCollision_struct = readMessages(ftCollision_bag,'DataFormat','struct');
    if isempty(ftCollision_struct)
        % recordings 2-12 recorded with different topic
        ftCollision_bag = select(bagft,'Topic','/rokubimini_cosmo/ft_sensor/readings');
        ftCollision_struct = readMessages(ftCollision_bag,'DataFormat','struct');
        recReadings = 1;
    else
        recReadings = 0;
    end

    % time collision ft sensor
    timeftCol = ftCollision_bag.MessageList.Time;
    timeftCol = timeftCol - timeftCol(1,1);
    endIndftCol = size(timeftCol,1);

    % read values
    ftForceMeasCol = zeros(3,endIndftCol);
    for i = 1:endIndftCol
        if recReadings == 0
            ftForceMeasCol(1,i) = ftCollision_struct{i,1}.Wrench.Force.X;
            ftForceMeasCol(2,i) = ftCollision_struct{i,1}.Wrench.Force.Y;
            ftForceMeasCol(3,i) = ftCollision_struct{i,1}.Wrench.Force.Z;
        else
            ftForceMeasCol(1,i) = ftCollision_struct{i,1}.Wrench.Wrench.Force.X;
            ftForceMeasCol(2,i) = ftCollision_struct{i,1}.Wrench.Wrench.Force.Y;
            ftForceMeasCol(3,i) = ftCollision_struct{i,1}.Wrench.Wrench.Force.Z;
        end      
    end

    %% remove offset FT sensor collision

    n1 = find(timeftCol>n1sec,1);
    n2 = find(timeftCol>n2sec,1);
    offset(1,1) = mean(ftForceMeasCol(1,n1:n2));
    offset(2,1) = mean(ftForceMeasCol(2,n1:n2));
    offset(3,1) = mean(ftForceMeasCol(3,n1:n2));

    ftForceCol = zeros(3,endIndftCol);
    for i = 1:endIndftCol
        ftForceCol(1,i) = ftForceMeasCol(1,i)-offset(1,1);
        ftForceCol(2,i) = ftForceMeasCol(2,i)-offset(2,1);
        ftForceCol(3,i) = ftForceMeasCol(3,i)-offset(3,1);
    end
    
    %% shift time interval
    timeftShifted = timeftCol + timeShift;
    
    %% compute magnitude FT sensor ground truth force
    magFTForce = sqrt(ftForceCol(1,:).^2 + ftForceCol(2,:).^2 + ftForceCol(3,:).^2);
   
end

