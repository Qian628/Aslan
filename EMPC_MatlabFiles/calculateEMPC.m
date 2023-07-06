function calculateEMPC(modelType, model_param_, mpc_param_, operat_cond_, dt)
% define the model parameters
wheelbase = model_param_.wheelbase; % meters
mass_fl = model_param_.mass_fl; % kg
mass_fr = model_param_.mass_fr; % kg
mass_rl = model_param_.mass_rl; % kg
mass_rr = model_param_.mass_rr; % kg
cf = model_param_.cf; % N/rad
cr = model_param_.cr; % N/rad
steer_tau = model_param_.steer_tau; % seconds
steer_lim = model_param_.steer_lim; % deg
lateral_error_lim = model_param_.lateral_error_lim; % m

% Define which model to use

switch modelType
    case 'DynamicsBicycleModel'
        vehicleModel = DynamicsBicycleModel(wheelbase, mass_fl, mass_fr, mass_rl, mass_rr, cf, cr);
    case 'DynamicsBicycleModelWithDelay'
        vehicleModel = DynamicsBicycleModelWithDelay(wheelbase, steer_tau, mass_fl, mass_fr, mass_rl, mass_rr, cf, cr);
    case 'KinematicsBicycleModel'
        vehicleModel = KinematicsBicycleModel(wheelbase, steer_lim);
    case 'KinematicsBicycleModelWithDelay'
        vehicleModel = KinematicsBicycleModelWithDelay(wheelbase, steer_lim, steer_tau);
    otherwise
        error('Unknown model type');
end

% Get the system dimensions from the model
DIM_X = vehicleModel.dim_x;
DIM_U = vehicleModel.dim_u;
DIM_Y = vehicleModel.dim_y;

N = mpc_param_.N;  % number of prediction horizon

% Set up trajectory data
mpc_resampled_ref_traj.k = repmat(operat_cond_.curvature, 1, N);
mpc_resampled_ref_traj.vx = repmat(operat_cond_.velocity, 1, N);

% Initialize Q, R matrices
Q = zeros(DIM_Y);
Q(1,1) = mpc_param_.weight_lat_error;
Q(2,2) = mpc_param_.weight_heading_error; 
R = mpc_param_.weight_steering_input;

% Call calculateMPCMatrix
try
    [Aex, Bex, Wex, Cex, Qex, Rex, Urefex] = calculateMPCMatrix(mpc_resampled_ref_traj, mpc_param_, modelType, vehicleModel, Q, R, N, dt, DIM_X, DIM_U, DIM_Y);
    disp('MPC Matrix calculation completed successfully');
catch ME
    disp(['MPC Matrix calculation failed with error: ' ME.message]);
end

% Call calculateParametericQPFormulation1 to generate explicit mpc regions
try
    solution_pQP1 = calculateParametricQPFormulation1(Aex, Bex, Cex, Qex, Rex, Wex, Urefex, steer_lim, lateral_error_lim, N, DIM_U, DIM_X, modelType);
    disp('parametric QP formulation 1 calculation completed successfully');
catch ME
    disp(['parametric QP formulation 1 calculation failed with error: ' ME.message]);
end

% Call calculateParametericQPFormulation1_withFeedforward to generate explicit mpc regions
try
    solution_pQP1_ff = calculateParametricQPFormulation1_withFeedforward(Aex, Bex, Cex, Qex, Rex, Urefex, steer_lim, lateral_error_lim, N, DIM_U, DIM_X, modelType);
    disp('parametric QP formulation 1 with feedforward calculation completed successfully');
catch ME
    disp(['parametric QP formulation 1 with feedforward calculation failed with error: ' ME.message]);
end

% Call calculateParametericQPFormulation2 to generate explicit mpc regions
try
    solution_pQP2 = calculateParametricQPFormulation2(Aex, Bex, Cex, Qex, Rex, Wex, Urefex, steer_lim, lateral_error_lim, N, DIM_U, DIM_X, modelType);
    disp('parametric QP formulation 2 calculation completed successfully');
catch ME
    disp(['parametric QP formulation 2 calculation failed with error: ' ME.message]);
end

% Call calculateParametericQPFormulation2_withFeedforward to generate explicit mpc regions
try
    solution_pQP2_ff = calculateParametricQPFormulation2_withFeedforward(Aex, Bex, Cex, Qex, Rex, Urefex, steer_lim, lateral_error_lim, N, DIM_U, DIM_X, modelType);
    disp('parametric QP formulation 2 with feedforward calculation completed successfully');
catch ME
    disp(['parametric QP formulation 2 with feedforward calculation failed with error: ' ME.message]);
end

% convert the solution to matlab script
solution_pQP1.xopt.toMatlab(['empc_solution/empc_solution_pQP1_', modelType, '_', num2str(operat_cond_.velocity), '_', num2str(operat_cond_.curvature), '.m'], 'primal', 'obj')
solution_pQP2.xopt.toMatlab(['empc_solution/empc_solution_pQP2_', modelType, '_', num2str(operat_cond_.velocity), '_', num2str(operat_cond_.curvature), '.m'], 'primal', 'obj')
solution_pQP1_ff.xopt.toMatlab(['empc_solution/empc_solution_pQP1_ff_', modelType, '_', num2str(operat_cond_.velocity), '_', num2str(operat_cond_.curvature), '.m'], 'primal', 'obj')
solution_pQP2_ff.xopt.toMatlab(['empc_solution/empc_solution_pQP2_ff_', modelType, '_', num2str(operat_cond_.velocity), '_', num2str(operat_cond_.curvature), '.m'], 'primal', 'obj')

% figure(1)
% solution_pQP1.xopt.fplot('primal','position',1);
% xlabel('lateral error [m]');
% ylabel('heading error [rad]');
% zlabel('steering angle [rad]')
% title('pQP1')
% 
% figure(2)
% solution_pQP2.xopt.fplot('primal','position',DIM_X*N+1);
% xlabel('lateral error [m]');
% ylabel('heading error [rad]');
% zlabel('steering angle [rad]')
% title('pQP2')

end
