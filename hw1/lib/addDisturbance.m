%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Question 3a: Add disturbance to trajectory  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Input Shape: 
%      Please check the MATLAB Workspace after running it once

% Output Shape: 
%      disturbance_idx:    int
%      q_mid:              4x1 or length = 4
%

function [disturbance_idx, q_mid] = addDisturbance(optimal_solution_full, q_disturbance, disturbance_val)
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Extract the trajectory information
    Xopt = optimal_solution_full.Xopt;
    Topt = optimal_solution_full.Topt;
    disturbance_idx = ceil(length(Topt) / 2);
    q_mid = Xopt(:, disturbance_idx); 
    q_mid(q_disturbance) = q_mid(q_disturbance) + deg2rad(disturbance_val);
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % choose your own disturbance idx within the size
    % q_disturbance is the index of joint you want to disturb
    % Modify q_mid configuration to simulate a perturbation pushing the robot
    % arm in another configuration
end