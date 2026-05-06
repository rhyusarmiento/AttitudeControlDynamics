clc; clear; close all;
restoredefaultpath; savepath;
Kp_var = optimizableVariable('Kp', [0.001, 0.5]);
Kd_var = optimizableVariable('Kd', [0.01, 2.0]);

objective_func = @(vars) attitude_project(vars.Kp, vars.Kd);
results = bayesopt(objective_func, [Kp_var, Kd_var], ...
    'MaxObjectiveEvaluations', 5, ...
    'AcquisitionFunctionName', 'expected-improvement-plus');

best_Kp = results.XAtMinObjective.Kp;
best_Kd = results.XAtMinObjective.Kd;
fprintf('\nOptimization Complete!\n');
fprintf('Best Kp: %f\n', best_Kp);
fprintf('Best Kd: %f\n', best_Kd);