function [allJacobians, jacobiansFeet_struct] = read_jacobians(bag, name)

%   Read jacobians from the rosbag

    % gripper
    jacobianGripper_bag = select(bag,'Topic',strcat(name,'/jacobian_gripper'));
    jacobianGripper_struct = readMessages(jacobianGripper_bag,'DataFormat','struct');

    % wrist 2
    jacobianWrist2_bag = select(bag,'Topic',strcat(name,'/jacobian_wrist2'));
    jacobianWrist2_struct = readMessages(jacobianWrist2_bag,'DataFormat','struct');

    % wrist 1
    jacobianWrist1_bag = select(bag,'Topic',strcat(name,'/jacobian_wrist1'));
    jacobianWrist1_struct = readMessages(jacobianWrist1_bag,'DataFormat','struct');

    % forearm
    jacobianForearm_bag = select(bag,'Topic',strcat(name,'/jacobian_forearm'));
    jacobianForearm_struct = readMessages(jacobianForearm_bag,'DataFormat','struct');

    % upperarm
    jacobianUpperarm_bag = select(bag,'Topic',strcat(name,'/jacobian_upperarm'));
    jacobianUpperarm_struct = readMessages(jacobianUpperarm_bag,'DataFormat','struct');  

    % base
    jacobianBase_bag = select(bag,'Topic',strcat(name,'/jacobian_base'));
    jacobianBase_struct = readMessages(jacobianBase_bag,'DataFormat','struct');

    % feet
    jacobiansFeet_bag = select(bag,'Topic',strcat(name,'/jacobians_feet'));
    jacobiansFeet_struct = readMessages(jacobiansFeet_bag,'DataFormat','struct');

    % stack in cell
    allJacobians = cell(6,1);
    allJacobians{1,1} = jacobianGripper_struct;
    allJacobians{2,1} = jacobianWrist2_struct;
    allJacobians{3,1} = jacobianWrist1_struct;
    allJacobians{4,1} = jacobianForearm_struct;
    allJacobians{5,1} = jacobianUpperarm_struct;
    allJacobians{6,1} = jacobianBase_struct;

end
