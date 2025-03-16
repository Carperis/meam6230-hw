%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL,
%    Switzerland
%   Author: Aude Billard
%   email:   aude.billard@epfl.ch
%   website: lasa.epfl.ch
%
%                                                                         % 
% Modified by Nadia Figueroa on March 2025, University of Pennsylvania    %
% email: nadiafig@seas.upenn.edu                                          %
%                                                                         %
%   Permission is granted to copy, distribute, and/or modify this program
%   under the terms of the GNU General Public License, version 2 or any
%   later version published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful, but
%   WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%   Public License for more details
%  Name of the chapter:  Dynamical system based compliant control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    MIT Press book 
%    Learning for Adaptive and Reactive Robot Control
%    Chapter 9 - Obstacle Avoidance: Programming exercise 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize system
clear; close all; clc;
x_limits = [-5, 5];
y_limits = [-5, 5];
nb_gridpoints = 200;

% mesh domain
[x, y] = meshgrid(linspace(x_limits(1), x_limits(2), nb_gridpoints), ...
                  linspace(y_limits(1), y_limits(2), nb_gridpoints));

%% Compute and draw nominal linear DS
A = [-1, 0; ...
     0, -1];  % Linear DS with stable attractor 
b = [-3; 0];  
x0 = -inv(A) * b; % Attractor coordinate are at Ax+b=0 

ds_fun = @(x)(A*x + b);
data = feval(ds_fun, [reshape(x, 1, []); reshape(y, 1, [])]);
x_dot = reshape(data(1,:), nb_gridpoints, nb_gridpoints);
y_dot = reshape(data(2,:), nb_gridpoints, nb_gridpoints);

% Plot DS
figure('Color', [1 1 1]);
hold on; axis equal;
title('Nominal Linear DS','Interpreter','latex','FontSize',15);
plot_ds(x, y, x_dot, y_dot);
plot(x0(1), x0(2), 'r*');
legend('Nominal DS', 'Attractor', 'Location', 'SouthWest');
xlim(x_limits);
ylim(y_limits);
drawnow;

%% Modulated DS - Simple elliptic obstacle
figure('Color', [1 1 1]);
hold on; axis equal;

% Construct and plot ellipse
object_center = [1; 0];     % Coordinate of center of obstacle
ellipse_axes = [1; 4];    % half lengths of ellipse
theta = linspace(0, 2*pi);
x_object = ellipse_axes(1) * cos(theta) + object_center(1);
y_object = ellipse_axes(2) * sin(theta) + object_center(2);

% Set reference point (question 2.1.2.b)
reference_point = [1; 3]; % Set reference point to create reference direction

% Initialize variables
x_dot_mod = zeros(nb_gridpoints, nb_gridpoints);
y_dot_mod = zeros(nb_gridpoints, nb_gridpoints);

for i=1:size(x, 1)
    for j=1:size(x, 2)

        % Compute distance function \Gamma(x) for a ellipse obstacle
        [gamma(i,j), gradient_gamma] = gamma_ellipse(x(i,j), y(i,j), object_center, ellipse_axes);
  
        % Construct Reference-based Modulation Matrix
        [~, ~, M] = reference_modulation(x(i,j), y(i,j), reference_point, gamma(i,j), gradient_gamma);

        % Construct Normal-based Modulation Matrix 
        % ==> Uncomment to test normal-modulation on elliptic obstacle
        % [~, ~, M] = normal_modulation(gamma(i,j), gradient_gamma);

        % If we are outside the obstacle (can use code from circular_modulation):
        if gamma(i,j) > 1
           xd = [x_dot(i,j) y_dot(i,j)]';
           xd_mod = apply_modulation(xd, M);
           x_dot_mod(i,j) = xd_mod(1); y_dot_mod(i,j) = xd_mod(2);
        end
        
    end
end

% Plotting functions
title('Reference-based Modulated DS with one elliptic obstacle','Interpreter','latex','FontSize',15);
plot_with_gamma = 1; % <== To plot \Gamma(x) of obstacle
if plot_with_gamma
    plot_ds_gamma(x, y, x_dot_mod, y_dot_mod, gamma);
    plot(reference_point(1), reference_point(2), 'r+', 'LineWidth', 3);
else
    plot_ds(x, y, x_dot_mod, y_dot_mod);
    plot(reference_point(1), reference_point(2), 'y+', 'LineWidth', 3);
end
plot(x_object, y_object, 'b', 'LineWidth', 2);
plot(object_center(1), object_center(2), 'bd','LineWidth', 3);
plot(x0(1), x0(2), 'b+','LineWidth', 3);
legend('\Gamma(x)', 'Obstacle', 'Obstacle center', 'Reference Point', ...
    'Attractor', 'Location', 'SouthWest');
xlim(x_limits);
ylim(y_limits);

function [h, h_stream] = plot_ds(x, y, x_dot, y_dot)
[~, h] = contourf(x, y, sqrt(x_dot.^2 + y_dot.^2), 80);
set(h, 'LineColor', 'none');
colormap('summer');
c_bar = colorbar;
c_bar.Label.String = 'Absolute velocity';
h_stream = streamslice(x, y, x_dot,y_dot, 2, 'method', 'cubic');
set(h_stream, 'LineWidth', 1);
set(h_stream, 'color', [0. 0. 0.]);
set(h_stream, 'HandleVisibility', 'off');
end

function [h, h_stream] = plot_ds_gamma(x, y, x_dot, y_dot, gamma)
[~, h] = contour(x, y, gamma, 80);
colormap('summer');
c_bar = colorbar;
c_bar.Label.String = '\Gamma(x)';
h_stream = streamslice(x, y, x_dot,y_dot, 2, 'method', 'cubic');
set(h_stream, 'LineWidth', 1);
set(h_stream, 'color', [0 0 0]);
set(h_stream, 'HandleVisibility', 'off');
end