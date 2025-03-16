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
x_limits = [-5, 5];
y_limits = [-5, 5];
nb_gridpoints = 200;

% mesh domain
[x, y] = meshgrid(linspace(x_limits(1), x_limits(2), nb_gridpoints), ...
                  linspace(y_limits(1), y_limits(2), nb_gridpoints));

%% Compute and draw nominal linear DS
A = [-1, 0; ...
     0, -1];  % Linear DS with stable attractor 
b = [-2; -2];  
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

%% Modulated DS - Simple circular obstacle
figure('Color', [1 1 1]);
hold on; axis equal;

% Construct and plot circle 
object_center = [1.5; 1.5];  % Coordinate of center of obstacle
radius = 2;  % Obstacle Radius 
theta = linspace(0, 2*pi);
x_object = radius * cos(theta) + object_center(1);
y_object = radius * sin(theta) + object_center(2);

% Initialize variables
x_dot_mod = zeros(nb_gridpoints, nb_gridpoints);
y_dot_mod = zeros(nb_gridpoints, nb_gridpoints);

for i=1:size(x, 1)
    for j=1:size(x, 2)

        % Compute distance function \Gamma(x) for a circular obstacle
        [gamma, gradient_gamma] = gamma_circular(x(i,j), y(i,j), object_center, radius);
        
        % Construct Normal-based Modulation Matrix
        [~, ~, M] = normal_modulation(gamma, gradient_gamma);
       
        % If we are outside the obstacle apply modulation:
        if gamma >= 1
           xd = [x_dot(i,j) y_dot(i,j)]';
           xd_mod = apply_modulation(xd, M);
           x_dot_mod(i,j) = xd_mod(1); y_dot_mod(i,j) = xd_mod(2);
        end
        
    end
end

% Plotting functions
title('Normal-based Modulated DS with one circular obstacle','Interpreter','latex','FontSize',15);
plot_ds(x, y, x_dot_mod, y_dot_mod);
plot(x_object, y_object, 'b', 'LineWidth', 2);
plot(object_center(1), object_center(2), 'bd');
plot(x0(1), x0(2), 'r*');
legend('Modulated DS', 'Obstacle', 'Obstacle center', ...
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
