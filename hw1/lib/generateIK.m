%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Task 5: Feasible Trajectory Dataset Generation via Iterative Optimization  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Input Shape: 
%      Please check the MATLAB Workspace after running it once

% Output Shape: 
%      trajectory:    8xN (N is the iteration steps you took to reach)
%

function trajectory = generateIK(robot, lambda, q0, targetPosition, maxJointSpeed, toleranceDistance, dt)
    trajectory = []; % 8xN array with 4 joints position and 4 joints speed

    %%%%%%%%%%%%%%%%%%%%%%%%%
    q = q0;
    position = robot.fastForwardKinematic(q);
    joint_positions = [];
    joint_velocities = [];
    
    while norm(position - targetPosition) > toleranceDistance
        J = robot.fastJacobian(q);
        % J_plus = pinv(J);
        J_plus = J' * inv(J * J' + lambda^2 * eye(size(J, 1)));
        delta_x = targetPosition - position;
        q_dot = J_plus * (delta_x / norm(delta_x));
        scale_factor = min(1, maxJointSpeed / max(abs(q_dot)));
        q_dot = q_dot * scale_factor;
        q = q + q_dot * dt;
        position = robot.fastForwardKinematic(q);
        joint_positions = [joint_positions, q];
        joint_velocities = [joint_velocities, q_dot];
    end
    trajectory = [joint_positions; joint_velocities];
    %%%%%%%%%%%%%%%%%%%%%%%%%
end