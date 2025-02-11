%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Question 3b: Generate complete trajectory  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Input Shape: 
%      Please check the MATLAB Workspace after running it once

% Output Shape: 
%      optimal_solution_final:    
%      should include fields same as optimal_solution_full and optimal_solution_after_disturbance (Topt, Xopt, Yopt, MVopt)
%

function optimal_solution_final = generateTraj(disturbance_idx, optimal_solution_full, optimal_solution_after_disturbance)
    optimal_solution_final = [];
    %%%%%%%%%%%%%%%%%%%%%%%%%
    Topt_full = optimal_solution_full.Topt(1:disturbance_idx);
    Xopt_full = optimal_solution_full.Xopt(:, 1:disturbance_idx);
    Yopt_full = optimal_solution_full.Yopt(:, 1:disturbance_idx);
    MVopt_full = optimal_solution_full.MVopt(:, 1:disturbance_idx);

    Topt_after = optimal_solution_after_disturbance.Topt;
    Topt_after = Topt_after + Topt_full(end); 
    Xopt_after = optimal_solution_after_disturbance.Xopt;
    Yopt_after = optimal_solution_after_disturbance.Yopt;
    MVopt_after = optimal_solution_after_disturbance.MVopt;

    Topt_final = [Topt_full, Topt_after];
    Xopt_final = [Xopt_full, Xopt_after];
    Yopt_final = [Yopt_full, Yopt_after];
    MVopt_final = [MVopt_full, MVopt_after];

    Topt_final = Topt_final';
    optimal_solution_final.Topt = Topt_final;
    optimal_solution_final.Xopt = Xopt_final;
    optimal_solution_final.Yopt = Yopt_final;
    optimal_solution_final.MVopt = MVopt_final;
    %%%%%%%%%%%%%%%%%%%%%%%%%
end
