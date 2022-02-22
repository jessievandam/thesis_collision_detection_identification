function [force, forceLPF, magEstForce, magEstForceLPF] = estimate_base_and_arm_force_detection(torques,...
                            allJacobians, jacobiansFeet_struct, endInd, fc, timeVec)

%   Estimation of base and arm force with MBO external joint torques

    %% estimate force
   
    % pre allocation
    force = cell(2,1);
    stackedFeetJacobian = zeros(12, 24);
    
    for h = 1:2 % loop over base and arm jacobian
        for i = 1:endInd

            % if h=1, arm jacobian, if h=2, base jacobian
            jacobianColBody = allJacobians{3*h,1}{i,1}.Data(1:end); % colliding body jacobian
            jacobiansFeet = jacobiansFeet_struct{i,1}.Data(1:end); 
            for j = 1:12
                stackedFeetJacobian(j,:) = jacobiansFeet((24*(j-1)+1):(24*j));
            end
             stackedJacobian = [stackedFeetJacobian; 
                            jacobianColBody(1:24)'; jacobianColBody(25:48)'; jacobianColBody(49:72)'; 
                            jacobianColBody(73:96)'; jacobianColBody(97:120)'; jacobianColBody(121:144)'];

            % estimate wrench
            wrench = pinv(stackedJacobian')*torques(:,i);     
            
            % extract force
            force{h,1}(:,i) = wrench(13:15,1);
        end
    end
    
    %% apply LPF to force when arm is moving
    
    sysLPF = tf([0.0, fc*2*pi], [1.0, fc*2*pi]);
    forceLPF = cell(2,1);
    for h = 1:2
        forceLPF{h,1}(1,:) = lsim(sysLPF,force{h,1}(1,:),timeVec);
        forceLPF{h,1}(2,:) = lsim(sysLPF,force{h,1}(2,:),timeVec);
        forceLPF{h,1}(3,:) = lsim(sysLPF,force{h,1}(3,:),timeVec);
    end

    %% compute magnitude of colliding body forces
    
    % force magnitudes
    magEstForce = cell(2,1);
    magEstForceLPF = cell(2,1);
    for h = 1:2
        magEstForce{h,1} = sqrt(force{h,1}(1,:).^2 + force{h,1}(2,:).^2 + force{h,1}(3,:).^2);
        magEstForceLPF{h,1} = sqrt(forceLPF{h,1}(1,:).^2 + forceLPF{h,1}(2,:).^2 + forceLPF{h,1}(3,:).^2);
    end

end