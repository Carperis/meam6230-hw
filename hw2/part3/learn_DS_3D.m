%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file is a template file to import 3D dataset to learn a DS from    %
% trajectories, and exporting the DS to be deployed on the Panda robot.   %
%  
% ====>> You should use functions from part 2 of this homework!           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Import dependencies
close all; clear; clc
filepath = fileparts(which('learn_DS_3D.m'));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-ds-opt')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-sods-opt')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-phys-gmm')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-thirdparty')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-robot-simulation')));
addpath(genpath(fullfile(filepath, 'dataset')));
% cd(filepath); %<<== This might be necessary in some machines

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 1 (DATA LOADING): Choose among the predifined datasets %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load and convert 3D dataset
% Import from the dataset folder either:
% - Task 1: 'theoretical_DS_dataset.mat'
% - Task 2: 'MPC_train_dataset.mat'
% - Task 2: 'MPC_test_dataset.mat'
% - Task 3: '3D_Cshape_bottom_processed.mat'
% - Task 3: 'raw_demonstration_dataset.mat'

% load("theoretical_DS_dataset.mat"); % --> Modify me to load different datasets!!
% load("MPC_train_dataset.mat");
% load("MPC_test_dataset.mat");
% load("3D_Cshape_bottom.mat")
% load("3D_Cshape_bottom_processed.mat")
load("raw_demonstration_dataset.mat")
% usingSEDS --> Modify me if you will use seds or lpvds! 
% (necessary for parameter storing method used in robot_DS_control.m)
usingSEDS = false;
% filter --> Modify me if you want to pre-process the datasets 
% (relevant for task 3)
filter = true;

% All code below is used to extract trajectories in format amenable to
% learning the DS with the codes provided in part 2 .m scripts
nTraj = size(trajectories, 3);
nPoints = size(trajectories, 2);

Data = [];
attractor = zeros(3, 1);
x0_all = zeros(3, nTraj);

% When filter = true the next lines of code will apply a savitzky golay
% filter to your data (this is recommended for raw human demonstrations)
for i = 1:nTraj
    if ismember(i,[5,7,11])
        continue;
    end
    traj = trajectories(:,:,i);
    % att = [0; 0; 0];
    % vel_samples = 5; vel_size = 0.75;
    % plot_reference_trajectories_DS(traj, att, vel_samples, vel_size);
    nCols = size(traj, 2);
    nTrim = 10;
    traj = traj(:, nTrim:nCols-nTrim);
    if filter
        % Filter Trajectories and Compute Derivativess with Savitzky Golay filter
        %   traj: The trajectory you want to filter
        %   sample_step: subsample the traj before filtering
        %   nth_order :     max order of the derivatives 
        %   n_polynomial :  Order of polynomial fit
        %   window_size :   Window length for the filter
        traj = sgolay_filter_smoothing(trajectories(:,:,i), 5, 1, 3, 10);
    end

    Data = [Data traj];
    x0_all(:,i) = traj(1:3,1);
    attractor = attractor + traj(1:3,end);
end
attractor = attractor / nTraj;

% Normalizing dataset attractor position
M = size(Data, 1) / 2; 
Data(1:M,:) = Data(1:M,:) - attractor;
x0_all = x0_all - attractor;
att = [0; 0; 0];

% Plot position/velocity Trajectories
vel_samples = 5; vel_size = 0.75; 
[h_data, h_att, ~] = plot_reference_trajectories_DS(Data, att, vel_samples, vel_size);

% Extract Position and Velocities
M = size(Data,1) / 2;    
Xi_ref = Data(1:M,:);
Xi_dot_ref  = Data(M+1:end,:);   
axis_limits = axis;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%    Step 2: ADD YOUR CODE BELOW TO LEARN 3D DS      %%
%% vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %%%%%

tStart = cputime;

if usingSEDS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%  SEDS Implementation for 3D %%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % GMM Estimation Options
    est_options = [];
    est_options.type        = 1;   % GMM Estimation Alorithm Type
    est_options.maxK        = 10;  % Maximum Gaussians for Type 1/2
    est_options.do_plots    = 1;   % Plot Estimation Statistics
    est_options.fixed_K     = [];  % Fix K and estimate with EM
    est_options.sub_sample  = 1;   % Size of sub-sampling of trajectories

    do_ms_bic = 1;
    if do_ms_bic
        [Priors0, Mu0, Sigma0] = fit_gmm([Xi_ref; Xi_dot_ref], [], est_options);
        nb_gaussians = length(Priors0);
    else
        nb_gaussians = 3; 
    end

    % Initialize SEDS parameters
    init_with_options = 0;

    if ~init_with_options
        % Run Algorithm 1 from Chapter 3 (Get an initial guess by deforming Sigma's)
        [Priors0, Mu0, Sigma0] = initialize_SEDS([Xi_ref; Xi_dot_ref], nb_gaussians);
    else
        % Run Algorithm 2 from Chapter 3 (Get an initial guess by optimizing
        % each K-th Gaussian function wrt. stability constraints independently
        clear init_options;
        init_options.tol_mat_bias  = 10^-4;
        init_options.tol_stopping  = 10^-10;
        init_options.max_iter      = 500;
        init_options.objective     = 'likelihood';
        [Priors0, Mu0, Sigma0] = initialize_SEDS([Xi_ref; Xi_dot_ref], nb_gaussians, init_options);
    end
    
    % SEDS Options
    options.tol_mat_bias  = 10^-4;
    options.display       = 1;
    options.tol_stopping  = 10^-10;
    options.max_iter      = 100;
    options.objective     = 'likelihood';  % 'mse'|'likelihood'
    % options.objective     = 'mse';         % 'mse'|'likelihood'

    % Run SEDS solver
    [Priors, Mu, Sigma] = SEDS_Solver(Priors0, Mu0, Sigma0, [Xi_ref; Xi_dot_ref], options);

    % Create DS function
    ds_seds = @(x) GMR_SEDS(Priors, Mu, Sigma, x - repmat(att, [1 size(x,2)]), 1:3, 4:6);
    
else
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%  LPV-DS Implementation for 3D %%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % GMM Estimation Options
    % 0: Physically-Consistent CRP-GMM (Collapsed Gibbs Sampler)
    % 1: GMM-EM Model Selection via BIC
    % 2: CRP-GMM (Collapsed Gibbs Sampler)
    est_options = [];
    est_options.type             = 0;   % GMM Estimation Algorithm Type 
    % If algo 1 selected:
    est_options.maxK             = 15;  % Maximum Gaussians for Type 1
    est_options.fixed_K          = [];  % Fix K and estimate with EM for Type 1
    % If algo 0 or 2 selected:
    est_options.samplerIter      = 30;  % Maximum Sampler Iterations
                                        % For type 0: 20-50 iter is sufficient
                                        % For type 2: >100 iter are needed               
    est_options.do_plots         = 1;   % Plot Estimation Statistics
    
    nb_data = length(Data);
    sub_sample = 1;
    if nb_data > 1500
        sub_sample = 8;    
    elseif nb_data > 1000
        sub_sample = 4;
    elseif nb_data > 500
        sub_sample = 2;
    end
     l_sensitivity = 2;
    est_options.sub_sample       = sub_sample;

    % Metric Hyper-parameters (for algo 0)
    est_options.estimate_l       = 1;   % '0/1' Estimate the lengthscale, if set to 1
    est_options.l_sensitivity    = l_sensitivity;   % lengthscale sensitivity [1-10->>100]
                                        % Default value is set to '2' as in the
                                        % paper, for very messy, close to
                                        % self-intersecting trajectories, we
                                        % recommend a higher value
 
    est_options.length_scale     = [];  % if estimate_l=0 you can define your own
                                        % l, when setting l=0 only
                                        % directionality is taken into account

    % Fit GMM to position data
    [Priors, Mu, Sigma] = fit_gmm(Xi_ref, Xi_dot_ref, est_options);
    
    % Create GMM structure
    clear ds_gmm; 
    ds_gmm.Mu = Mu; 
    ds_gmm.Sigma = Sigma; 
    ds_gmm.Priors = Priors; 

    % Adjust covariance matrices
    adjusts_C  = 1;
    if adjusts_C  == 1
        if M == 2
            tot_dilation_factor = 1; rel_dilation_fact = 0.2;
        elseif M == 3
            tot_dilation_factor = 1; rel_dilation_fact = 0.75;
        end
        Sigma_ = adjust_Covariances(ds_gmm.Priors, ds_gmm.Sigma, tot_dilation_factor, rel_dilation_fact);
        ds_gmm.Sigma = Sigma_;
    end

    % LPV-DS Optimization Options
    constr_type = 2;      % 0:'convex':     A' + A < 0 (Same as SEDS, convex)
                          % 1:'non-convex': A'P + PA < 0 (Estimate P, nonconvex)
                          % 2:'non-convex': A'P + PA < Q (Pre-estimates P, Q <= -eps*I explicitly constrained)                                 
    init_cvx    = 1;      % 0/1: initialize non-cvx problem with cvx solution, normally this is not needed
                          % but for some datasets with lots of points or highly non-linear it helps the 
                          % non-convex optimization converge faster. However, in some cases it might  
                          % bias the non-cvx problem too much and reduce
                          % reproduction accuracy.
        
    % Learn P matrix (WSAQF)
    if constr_type == 0 || constr_type == 1
        P_opt = eye(M);
    else
        % P-matrix learning (Data shifted to the origin)
        % Assuming origin is the attractor (optimization works better generally)
        [Vxf] = learn_wsaqf(Data);
        P_opt = Vxf.P;
        fprintf('P matrix pre-estimated.\n');
    end

    % Optimize LPV-DS
    [A_k, b_k, ~] = optimize_lpv_ds_from_data(Data, att, constr_type, ds_gmm, P_opt, init_cvx);
    if constr_type == 1
        [A_k, b_k, P_est] = optimize_lpv_ds_from_data(Data, zeros(M,1), constr_type, ds_gmm, P_opt, init_cvx);
        ds_lpv = @(x) lpv_ds(x-repmat(att, [1 size(x,2)]), ds_gmm, A_k, b_k);
    else
        [A_k, b_k, ~] = optimize_lpv_ds_from_data(Data, att, constr_type, ds_gmm, P_opt, init_cvx);
        ds_lpv = @(x) lpv_ds(x, ds_gmm, A_k, b_k);
    end

end

%%   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ %%%
%%    Step 2: ADD YOUR CODE ABOVE TO LEARN 3D DS      %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if usingSEDS
    % Compute DS parameters: A_k and b_k for each Gaussian (K > 1)
    K = size(Mu, 2);
    A_k_all = zeros(3, 3, K);
    b_k_all = zeros(3, K);
    
    for k = 1:K
        % Extract mean for component k
        mu_x_k = Mu(1:3, k);
        mu_dot_k = Mu(4:6, k);
        
        % Extract covariance for component k (6x6 matrix)
        Sigma_k = Sigma(:, :, k);
        Sigma_x = Sigma_k(1:3, 1:3);
        Sigma_dotx = Sigma_k(4:6, 1:3);
        
        % Compute DS parameters for component k
        A_k_all(:, :, k) = Sigma_dotx / Sigma_x;
        b_k_all(:, k) = mu_dot_k - A_k_all(:, :, k) * mu_x_k;
    end
    
    % Display the per-component DS parameters
    for k = 1:K
        fprintf('Component %d:\n', k);
        fprintf('[A_k]: \n');
        disp(A_k_all(:, :, k));
        fprintf('[b_k]: \n');
        disp(b_k_all(:, k));
    end
end

% Plot Resulting DS
ds_plot_options = [];
ds_plot_options.sim_traj  = 1;            % Simulate trajectories
ds_plot_options.x0_all    = x0_all;       % Initial points
ds_plot_options.init_type = 'ellipsoid';  % 3D streamline init
ds_plot_options.nb_points = 30;           % Number of streamlines
ds_plot_options.plot_vol  = 0;            % Plot 3D volume

if usingSEDS
    [~, hs, hr, x_sim] = visualizeEstimatedDS(Data(1:M,:), ds_seds, ds_plot_options);
    title('SEDS: Reference vs Learned Trajectories');
else
    [~, hs, hr, x_sim] = visualizeEstimatedDS(Data(1:M,:), ds_lpv, ds_plot_options);
    title('LPV-DS: Reference vs Learned Trajectories');
end

% Attractor marker
scatter3(att(1), att(2), att(3), 150, [0 0 0], 'd', 'Linewidth', 2); 
hold on;

% Enhance plot
grid on; axis equal;
legend('Dataset trajectories', 'Learned DS')

if usingSEDS
    ds = ds_seds;
else
    ds = ds_lpv;
end

% 1. Compute RMSE on training data
rmse = mean(rmse_error(ds, Xi_ref, Xi_dot_ref));
fprintf('[RMSE] Velocity error: %.4f m/s\n', rmse);

% 2. Compute e_dot (velocity direction similarity)
edot = mean(edot_error(ds, Xi_ref, Xi_dot_ref));
fprintf('[e_dot] Velocity deviation: %.4f rad\n', edot);

% 3. Display computation time
tEnd = cputime - tStart;
fprintf('[Time] Total training time: %.2f seconds\n', tEnd);

% 4. Compute DTWD between reference and reproduced trajectories
if exist('x_sim','var') && ~isempty(x_sim)
    nb_traj = size(x_sim, 3);
    ref_traj_leng = size(Xi_ref,2)/nb_traj;
    dtwd = zeros(1,nb_traj);
    
    for n=1:nb_traj
        start_id = round(1+(n-1)*ref_traj_leng);
        end_id = round(n*ref_traj_leng);
        dtwd(n) = dtw(x_sim(:,:,n)', Xi_ref(:,start_id:end_id)', 20);
    end
    
    fprintf('[DTWD] Shape similarity: %.4f ± %.4f\n', mean(dtwd), std(dtwd));
else
    fprintf('[DTWD] Not computed - enable trajectory simulation\n');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 3 (SAVE DS): Save learned DS parameters for robot control %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save DS for simulation using 'DS_control.m'
filename = strcat(filepath,'/ds_control.mat');
if usingSEDS
    ds_control = @(x) ds_seds(x - attractor);
    save('ds_control.mat', "ds_control", "attractor", "Priors", "Mu", "Sigma", "att", "M")
else
    ds_control = @(x) ds_lpv(x - attractor);
    save('ds_control.mat', "ds_control", "attractor", "ds_gmm", "A_k", "b_k", "att")
end