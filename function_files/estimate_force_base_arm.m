function [force, forceLPF, magEstForce, magEstForceLPF, moment] = estimate_force_base_arm(torquesStacked,...
                allJacobians, jacobiansFeet_struct, endInd, fc, timeVec)

%   Estimation of wrench with Jacobians and external joint torques

    %% estimate wrench
    
    % params
    K = size(torquesStacked,1);
    H = 2;
    
    % pre allocation
    force = cell(K,H);
    moment = cell(K,H);
    for k = 1:K % loop over all external torques
        for h = 1:H % loop over all jacobians
            force{k,h} = zeros(3, endInd);
            moment{k,h} = zeros(3, endInd);
        end
    end
    stackedFeetJacobian = zeros(12, 24);

    for k = 1:K % loop over all external torques
        for h = 1:H % loop over all jacobians
            for i = 1:endInd 

                % read out torques calculated by MBO, GM, direct
                torques = torquesStacked{k,1}(:,i);

                % colliding body jacobian
                jacobianColBody = allJacobians{h*3,1}{i,1}.Data(1:end); 
                jacobiansFeet = jacobiansFeet_struct{i,1}.Data(1:end); 
                for j = 1:12
                    stackedFeetJacobian(j,:) = jacobiansFeet((24*(j-1)+1):(24*j));
                end
                 stackedJacobian = [stackedFeetJacobian; 
                                jacobianColBody(1:24)'; jacobianColBody(25:48)'; jacobianColBody(49:72)'; 
                                jacobianColBody(73:96)'; jacobianColBody(97:120)'; jacobianColBody(121:144)'];
                
                % estimate stacked force vector feet and force,moment colliding body
                wrench = pinv(stackedJacobian')*torques; 
                force{k,h}(:,i) = wrench(13:15); 
                moment{k,h}(:,i) = wrench(16:18); 
            end
        end
    end
    
    %% apply LPF to wrench during trotting
    
    sysLPF = tf([0.0, fc*2*pi], [1.0, fc*2*pi]);
    forceLPF = cell(K,H);
    for k = 1:K
        for h = 1:H 
            forceLPF{k,h}(1,:) = lsim(sysLPF,force{k,h}(1,:),timeVec);
            forceLPF{k,h}(2,:) = lsim(sysLPF,force{k,h}(2,:),timeVec);
            forceLPF{k,h}(3,:) = lsim(sysLPF,force{k,h}(3,:),timeVec);
        end
    end
    
    %% compute magnitude of colliding body forces
    
    % force magnitudes
    magEstForce = cell(K,H);
    magEstForceLPF = cell(K,H);
    for k = 1:K % loop over all external torques
        for h = 1:H % loop over all jacobians
            magEstForce{k,h} = sqrt(force{k,h}(1,:).^2 + force{k,h}(2,:).^2 + force{k,h}(3,:).^2);
            magEstForceLPF{k,h} = sqrt(forceLPF{k,h}(1,:).^2 + forceLPF{k,h}(2,:).^2 + forceLPF{k,h}(3,:).^2);
        end
    end

end

