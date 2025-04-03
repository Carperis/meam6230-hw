%% %%%%%%%%%%%%%%%%%%
%%  unstable_point %%
%%%%%%%%%%%%%%%%%%%%%
% See Lecture 8 Slide 15
% Input Shape: 
%      x:                  2x1 vector (2-dimensional state vector x)
%      xd:                 2x1 vector (2-dimensional vectors f(x))
%      x0:                 2x1 vector (coordinate of modulator)
%      sigma:              scalar (Kernel width for effect of modulator)

% Output Shape: 
%      gamma:              scalar (activation function)
%      v:                  2x1 matrix (modulated dynamics)

function [gamma, v] = unstable_point(x, xd, x0, sigma)   
    gamma = 0; v = [0;0];
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Calculate distance from x to x0
    dx = x - x0;
    dist = norm(dx);
    
    % Compute activation function gamma
    gamma = exp(-dist^2 / sigma^2);
    
    % Create destabilizing matrix (flip x-component)
    M = [1 - 2*gamma, 0; 
        0, 1 - 2*gamma];
    
    % Modulate the velocity (creates repulsion in x-direction)
    v = M * xd;
end

