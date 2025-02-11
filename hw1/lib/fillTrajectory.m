%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Task 4: Compute closed-form trajectory  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Input Shape: 
%      Please check the MATLAB Workspace after running it once

% Output Shape: 
%      cartesianTrajectory:    3x50
%

function cartesianTrajectory = fillTrajectory(time, initial_position, waypoints, target_position)
    % Compute trajectory based on third order polynomial
    cartesianTrajectory = nan(3, length(time));
    %%%%%%%%%%%%%%%%%%%%%%%%%
    n = length(time);
    t0 = 0;
    t1 = time(round(n/3));
    t2 = time(round(2*n/3));
    tf = time(end);
    wp1 = waypoints(:, 1);
    wp2 = waypoints(:, 2);
    for dim = 1:3
        P = [initial_position(dim); wp1(dim); wp2(dim); target_position(dim)];
        % T = [t0; t1; t2; tf];
        A = [1, t0, t0^2, t0^3;
             1, t1, t1^2, t1^3;
             1, t2, t2^2, t2^3;
             1, tf, tf^2, tf^3];
        coeffs = A \ P;
        cartesianTrajectory(dim, :) = coeffs(1) + coeffs(2)*time + coeffs(3)*time.^2 + coeffs(4)*time.^3;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%
end