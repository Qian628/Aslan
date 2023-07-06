clear all
clc
close all

% define the model parameters
model_param_.wheelbase = 1.686; % meters
model_param_.mass_fl = 108.75; % kg
model_param_.mass_fr = 108.75; % kg
model_param_.mass_rl = 108.75; % kg
model_param_.mass_rr = 108.75; % kg
model_param_.cf = 155494.663; % N/rad
model_param_.cr = 155494.663; % N/rad
model_param_.steer_tau = 0.3; % seconds
model_param_.steer_lim = 40; % deg
model_param_.lateral_error_lim = 1; % m

% Define which model to use
modelType = 'KinematicsBicycleModelWithDelay'; % 'DynamicsBicycleModel', 'DynamicsBicycleModelWithDelay', 'KinematicsBicycleModel', 'KinematicsBicycleModelNoDelay'

switch modelType
    case 'DynamicsBicycleModel'
        vehicleModel = DynamicsBicycleModel(model_param_.wheelbase, model_param_.mass_fl, model_param_.mass_fr, model_param_.mass_rl, model_param_.mass_rr, model_param_.cf, model_param_.cr);
    case 'DynamicsBicycleModelWithDelay'
        vehicleModel = DynamicsBicycleModelWithDelay(model_param_.wheelbase, model_param_.steer_tau, model_param_.mass_fl, model_param_.mass_fr, model_param_.mass_rl, model_param_.mass_rr, model_param_.cf, model_param_.cr);
    case 'KinematicsBicycleModel'
        vehicleModel = KinematicsBicycleModel(model_param_.wheelbase, model_param_.steer_lim);
    case 'KinematicsBicycleModelWithDelay'
        vehicleModel = KinematicsBicycleModelWithDelay(model_param_.wheelbase, model_param_.steer_lim, model_param_.steer_tau);
    otherwise
        error('Unknown model type');
end

% Get the system dimensions from the model
DIM_X = vehicleModel.dim_x;
DIM_U = vehicleModel.dim_u;
DIM_Y = vehicleModel.dim_y;

% define reference speed and curvature on the reference trajecotry 
velocity = 7;  % m/s, replace with your own value
curvature = 0.1;  % 1/m, replace with your own value

% define the time step
dt = 0.1;  % seconds, time step, replace with your own value

% Additional Parameters for calculateMPC function
mpc_param_.N = 70;  % number of time steps
N = mpc_param_.N;
mpc_param_.weight_lat_error = 0.1;
mpc_param_.weight_heading_error = 0;
mpc_param_.weight_steering_input = 1;
mpc_param_.weight_terminal_lat_error = 0.1;
mpc_param_.weight_terminal_heading_error = 0;
mpc_param_.weight_heading_error_squared_vel_coeff = 0;
mpc_param_.weight_steering_input_squared_vel_coeff = 0;
mpc_param_.weight_lat_jerk = 0;
mpc_param_.zero_ff_steer_deg = 2;

% Set up dummy trajectory data
mpc_resampled_ref_traj_all.k = repmat(curvature, 1, 10*N);
mpc_resampled_ref_traj_all.vx = repmat(velocity, 1, 10*N);

% Initialize Q, R matrices
Q = zeros(DIM_Y);
Q(1,1) = mpc_param_.weight_lat_error;
Q(2,2) = mpc_param_.weight_heading_error; 
% this is not corret for dynamicsmodels. need to be corrected
R = mpc_param_.weight_steering_input;
 
% Set initial state x0
init_lateral_error = 0.1;
init_heading_error = 0;
Uref = vehicleModel.calculateReferenceInput(curvature);
x0_qp1 = zeros(DIM_X, 1);
x0_qp1(1) = init_lateral_error;
x0_qp1(2) = init_heading_error;
x0_qp1_ff = zeros(DIM_X, 1);
x0_qp1_ff(1) = init_lateral_error;
x0_qp1_ff(2) = init_heading_error;
x0_qp2 = zeros(DIM_X, 1);
x0_qp2(1) = init_lateral_error;
x0_qp2(2) = init_heading_error;
x0_qp2_ff = zeros(DIM_X, 1);
x0_qp2_ff(1) = init_lateral_error;
x0_qp2_ff(2) = init_heading_error;
x0_pQP1 = zeros(DIM_X, 1);
x0_pQP1(1) = init_lateral_error;
x0_pQP1(2) = init_heading_error;
x0_pQP1_ff = zeros(DIM_X, 1);
x0_pQP1_ff(1) = init_lateral_error;
x0_pQP1_ff(2) = init_heading_error;
x0_pQP2 = zeros(DIM_X, 1);
x0_pQP2(1) = init_lateral_error;
x0_pQP2(2) = init_heading_error;
x0_pQP2_ff= zeros(DIM_X, 1);
x0_pQP2_ff(1) = init_lateral_error;
x0_pQP2_ff(2) = init_heading_error;

% set simulation step
stepSpanN = N; 
for i = 1:stepSpanN
    % shift the reference traj
    mpc_resampled_ref_traj.k = mpc_resampled_ref_traj_all.k(i:N+i); 
    mpc_resampled_ref_traj.vx = mpc_resampled_ref_traj_all.vx(i:N+i); 
    
    % calculate the vehicle model matrices
    [Ad, Bd, Cd, Wd] = vehicleModel.calculateDiscreteMatrix(mpc_resampled_ref_traj.vx(1), mpc_resampled_ref_traj.k(1), dt);
    
    % Call calculateReferenceInput
        if strcmp(modelType, 'DynamicsBicycleModel') || strcmp(modelType, 'DynamicsBicycleModelWithDelay')
            Uref = vehicleModel.calculateReferenceInput(mpc_resampled_ref_traj.vx(1), mpc_resampled_ref_traj.k(1));
        else
            Uref = vehicleModel.calculateReferenceInput(mpc_resampled_ref_traj.k(1));
        end
            Uref_all(:,i) = Uref; 

    % Call calculateMPCMatrix
    try
        [Aex, Bex, Wex, Cex, Qex, Rex, Urefex] = calculateMPCMatrix(mpc_resampled_ref_traj, mpc_param_, modelType, vehicleModel, Q, R, N, dt, DIM_X, DIM_U, DIM_Y);
        disp('MPC Matrix calculation completed successfully');
    catch ME
        disp(['MPC Matrix calculation failed with error: ' ME.message]);
    end

    % Call calculateQPFormulation1
    try
        Uex_qp1 = calculateQPFormulation1(x0_qp1, Aex, Bex, Cex, Qex, Rex, Wex, Urefex, steer_lim, lateral_error_lim, N, DIM_U, DIM_X,modelType);
        disp('QP formulation 1 calculation completed successfully');
    catch ME
        disp(['QP formulation 1 calculation failed with error: ' ME.message]);
    end
        % Update the state based on the calculated control action (use first action from Uex_qp1)
        x_updated_qp1 = updateState(Ad, Bd, Wd, x0_qp1, Uex_qp1(1));
        % store the computed control actions and updated states for later analysis
        Uex_all_qp1(:, i) = Uex_qp1(2,DIM_U);
        if i == 1
        x_all_qp1(:, i) = x0_qp1;
        else
        x_all_qp1(:, i) = x_updated_qp1;
        end
        % Update initial state x0 with new state
        x0_qp1 = x_updated_qp1;

     % Call calculateQPFormulation1_withFeedforward
    try
        switch modelType
            case 'DynamicsBicycleModel'
                delta_Uex_qp1_ff = calculateQPFormulation1_withFeedforward(x0_qp1_ff, Aex, Bex, Cex, Qex, Rex, Urefex, steer_lim, lateral_error_lim, N, DIM_U, DIM_X, modelType);
            case 'DynamicsBicycleModelWithDelay'
                delta_Uex_qp1_ff = calculateQPFormulation1_withFeedforward(x0_qp1_ff - Uref*[0;0;0;0;1], Aex, Bex, Cex, Qex, Rex, Urefex, steer_lim, lateral_error_lim, N, DIM_U, DIM_X, modelType);
            case 'KinematicsBicycleModel'
                delta_Uex_qp1_ff = calculateQPFormulation1_withFeedforward(x0_qp1_ff, Aex, Bex, Cex, Qex, Rex, Urefex, steer_lim, lateral_error_lim, N, DIM_U, DIM_X, modelType);
            case 'KinematicsBicycleModelWithDelay'
                delta_Uex_qp1_ff = calculateQPFormulation1_withFeedforward(x0_qp1_ff - Uref*[0;0;1], Aex, Bex, Cex, Qex, Rex, Urefex, steer_lim, lateral_error_lim, N, DIM_U, DIM_X, modelType);
        end
        
        disp('QP formulation 1 with feedforward calculation completed successfully');
    catch ME
        disp(['QP formulation 1 with feedforward calculation failed with error: ' ME.message]);
    end
        Uex_qp1_ff = Urefex + delta_Uex_qp1_ff;
        % Update the state based on the calculated control action (use first action from Uex_qp1)
        x_updated_qp1_ff = updateState(Ad, Bd, Wd, x0_qp1_ff, Uex_qp1_ff(1));
        % store the computed control actions and updated states for later analysis
        Uex_all_qp1_ff(:, i) = Uex_qp1_ff(2,DIM_U);
        if i == 1
        x_all_qp1_ff(:, i) = x0_qp1_ff;
        else
        x_all_qp1_ff(:, i) = x_updated_qp1_ff;
        end
        % Update initial state x0 with new state
        x0_qp1_ff = x_updated_qp1_ff;

     % Call calculateQPFormulation2
    try
        Uex_qp2 = calculateQPFormulation2(x0_qp2, Aex, Bex, Cex, Qex, Rex, Wex, Urefex, steer_lim, lateral_error_lim, N, DIM_U, DIM_X, modelType);
        disp('QP formulation 2 calculation completed successfully');
    catch ME
        disp(['QP formulation 2 calculation failed with error: ' ME.message]);
    end
        % Update the state based on the calculated control action (use first action from Uex_qp1)
        x_updated_qp2 = updateState(Ad, Bd, Wd, x0_qp2, Uex_qp2(1));
        % Optional: store the computed control actions and updated states for later analysis
        Uex_all_qp2(:, i) = Uex_qp2(2,DIM_U);
        if i == 1
        x_all_qp2(:, i) = x0_qp2;
        else
        x_all_qp2(:, i) = x_updated_qp2;
        end
        % Update initial state x0 with new state
        x0_qp2 = x_updated_qp2;

     % Call calculateQPFormulation2_withFeedforward
    try
        switch modelType
            case 'DynamicsBicycleModel'
                delta_Uex_qp2_ff = calculateQPFormulation2_withFeedforward(x0_qp2_ff, Aex, Bex, Cex, Qex, Rex, Urefex, steer_lim, lateral_error_lim, N, DIM_U, DIM_X, modelType);
            case 'DynamicsBicycleModelWithDelay'
                delta_Uex_qp2_ff = calculateQPFormulation2_withFeedforward(x0_qp2_ff - Uref*[0;0;0;0;1], Aex, Bex, Cex, Qex, Rex, Urefex, steer_lim, lateral_error_lim, N, DIM_U, DIM_X, modelType);
            case 'KinematicsBicycleModel'
                delta_Uex_qp2_ff = calculateQPFormulation2_withFeedforward(x0_qp2_ff, Aex, Bex, Cex, Qex, Rex, Urefex, steer_lim, lateral_error_lim, N, DIM_U, DIM_X, modelType);
            case 'KinematicsBicycleModelWithDelay'
                delta_Uex_qp2_ff = calculateQPFormulation2_withFeedforward(x0_qp2_ff - Uref*[0;0;1] , Aex, Bex, Cex, Qex, Rex, Urefex, steer_lim, lateral_error_lim, N, DIM_U, DIM_X, modelType);
        end
        
        disp('QP formulation 2 with feedforward calculation completed successfully');
    catch ME
        disp(['QP formulation 2 with feedforward calculation failed with error: ' ME.message]);
    end
        Uex_qp2_ff = Urefex + delta_Uex_qp2_ff;
        % Update the state based on the calculated control action (use first action from Uex_qp1)
        x_updated_qp2_ff = updateState(Ad, Bd, Wd, x0_qp2_ff, Uex_qp2_ff(1));
        % store the computed control actions and updated states for later analysis
        Uex_all_qp2_ff(:, i) = Uex_qp2_ff(2,DIM_U);
        if i == 1
        x_all_qp2_ff(:, i) = x0_qp2_ff;
        else
        x_all_qp2_ff(:, i) = x_updated_qp2_ff;
        end
        % Update initial state x0 with new state
        x0_qp2_ff = x_updated_qp2_ff;
        
     % Call empc_solution_pQP1
     switch modelType
        case 'DynamicsBicycleModel'
            [Uex_pQP1,k] = empc_solution_pQP1_DynamicsBicycleModel_7_0_1(x0_pQP1);
        case 'DynamicsBicycleModelWithDelay'
            [Uex_pQP1,k] = empc_solution_pQP1_DynamicsBicycleModelWithDelay_7_0_1(x0_pQP1);
        case 'KinematicsBicycleModel'
            [Uex_pQP1,k] = empc_solution_pQP1_KinematicsBicycleModel_7_0_1(x0_pQP1);
        case 'KinematicsBicycleModelWithDelay'
            [Uex_pQP1,k] = empc_solution_pQP1_KinematicsBicycleModelWithDelay_7_0_1(x0_pQP1);
     end
        %Uex_pQP1 = empc_solution_pQP1.xopt.feval(x0_pQP1,'primal');

        % Update the state based on the calculated control action (use first action from Uex_qp1)
        x_updated_pQP1 = updateState(Ad, Bd, Wd, x0_pQP1, Uex_pQP1(1));
        % Optional: store the computed control actions and updated states for later analysis
        Uex_all_pQP1(:, i) = Uex_pQP1(2,DIM_U);
        if i == 1
        x_all_pQP1(:, i) = x0_pQP1;
        else
        x_all_pQP1(:, i) = x_updated_pQP1;
        end
        % Update initial state x0 with new state
        x0_pQP1 = x_updated_pQP1;
     
     % Call empc_solution_pQP1_ff
     switch modelType
         case 'DynamicsBicycleModel'
            [delta_Uex_pQP1_ff,k] = empc_solution_pQP1_ff_DynamicsBicycleModel_7_0_1(x0_pQP1_ff);
        case 'DynamicsBicycleModelWithDelay'
            [delta_Uex_pQP1_ff,k] = empc_solution_pQP1_ff_DynamicsBicycleModelWithDelay_7_0_1(x0_pQP1_ff - Uref*[0;0;0;0;1]);
        case 'KinematicsBicycleModel'
            [delta_Uex_pQP1_ff,k] = empc_solution_pQP1_ff_KinematicsBicycleModel_7_0_1(x0_pQP1_ff );
        case 'KinematicsBicycleModelWithDelay'
            [delta_Uex_pQP1_ff,k] = empc_solution_pQP1_ff_KinematicsBicycleModelWithDelay_7_0_1(x0_pQP1_ff - Uref*[0;0;1]);
     end
        %delta_Uex_pQP1_ff = empc_solution_pQP1_ff.xopt.feval(x0_pQP1_ff,'primal');
        Uex_pQP1_ff = delta_Uex_pQP1_ff + Urefex;
        % Update the state based on the calculated control action (use first action from Uex_qp1)
        x_updated_pQP1_ff = updateState(Ad, Bd, Wd, x0_pQP1_ff, Uex_pQP1_ff(1));
        % Optional: store the computed control actions and updated states for later analysis
        Uex_all_pQP1_ff(:, i) = Uex_pQP1_ff(2,DIM_U);
        if i == 1
        x_all_pQP1_ff(:, i) = x0_pQP1_ff;
        else
        x_all_pQP1_ff(:, i) = x_updated_pQP1_ff;
        end
        % Update initial state x0 with new state
        x0_pQP1_ff = x_updated_pQP1_ff;

     % Call empc_solution_pQP2
     switch modelType
        case 'DynamicsBicycleModel'
            [Zex_pQP2,k] = empc_solution_pQP2_DynamicsBicycleModel_7_0_1(x0_pQP2);
        case 'DynamicsBicycleModelWithDelay'
            [Zex_pQP2,k] = empc_solution_pQP2_DynamicsBicycleModelWithDelay_7_0_1(x0_pQP2);
        case 'KinematicsBicycleModel'
            [Zex_pQP2,k] = empc_solution_pQP2_KinematicsBicycleModel_7_0_1(x0_pQP2);
        case 'KinematicsBicycleModelWithDelay'
            [Zex_pQP2,k] = empc_solution_pQP2_KinematicsBicycleModelWithDelay_7_0_1(x0_pQP2);
     end
        %Zex_pQP2 = empc_solution_pQP2.xopt.feval(x0_pQP2,'primal');
        Uex_pQP2 = Zex_pQP2(DIM_X*N+1:end);
        % Update the state based on the calculated control action (use first action from Uex_qp1)
        x_updated_pQP2 = updateState(Ad, Bd, Wd, x0_pQP2, Uex_pQP2(1));
        % Optional: store the computed control actions and updated states for later analysis
        Uex_all_pQP2(:, i) = Uex_pQP2(2,DIM_U);
        if i == 1
        x_all_pQP2(:, i) = x0_pQP2;
        else
        x_all_pQP2(:, i) = x_updated_pQP2;
        end
        % Update initial state x0 with new state
        x0_pQP2 = x_updated_pQP2;
     
     % Call empc_solution_pQP2_ff
     switch modelType
        case 'DynamicsBicycleModel'
            [Zex_pQP2_ff,k] = empc_solution_pQP2_ff_DynamicsBicycleModel_7_0_1(x0_pQP2_ff);
        case 'DynamicsBicycleModelWithDelay'
            [Zex_pQP2_ff,k] = empc_solution_pQP2_ff_DynamicsBicycleModelWithDelay_7_0_1(x0_pQP2_ff - Uref*[0;0;0;0;1]);
        case 'KinematicsBicycleModel'
            [Zex_pQP2_ff,k] = empc_solution_pQP2_ff_KinematicsBicycleModel_7_0_1(x0_pQP2_ff);
        case 'KinematicsBicycleModelWithDelay'
            [Zex_pQP2_ff,k] = empc_solution_pQP2_ff_KinematicsBicycleModelWithDelay_7_0_1(x0_pQP2_ff - Uref*[0;0;1]);
     end
        %Zex_pQP2_ff = empc_solution_pQP2_ff.xopt.feval(x0_pQP2_ff,'primal');
        delta_Uex_pQP2_ff = Zex_pQP2_ff(DIM_X*N+1:end);
        Uex_pQP2_ff = delta_Uex_pQP2_ff + Urefex;
        % Update the state based on the calculated control action (use first action from Uex_qp1)
        x_updated_pQP2_ff = updateState(Ad, Bd, Wd, x0_pQP2_ff, Uex_pQP2_ff(1));
        % Optional: store the computed control actions and updated states for later analysis
        Uex_all_pQP2_ff(:, i) = Uex_pQP2_ff(2,DIM_U);
        if i == 1
        x_all_pQP2_ff(:, i) = x0_pQP2_ff;
        else
        x_all_pQP2_ff(:, i) = x_updated_pQP2_ff;
        end
        % Update initial state x0 with new state
        x0_pQP2_ff = x_updated_pQP2_ff;
           
end

% Define the time step array for plotting
timeSteps = 0:dt:(stepSpanN-1)*dt;

% Plotting the results of QP1
figure(1);
subplot(2,1,1);
plot(timeSteps, Uex_all_qp1, 'LineWidth', 2);
hold on
plot(timeSteps, Uref_all, 'LineWidth', 2);
hold off
title('QP1: Control input Uex');
xlabel('Time (s)');
ylabel('Uex');

subplot(2,1,2);
plot(timeSteps, x_all_qp1, 'LineWidth', 2);
title('QP1: State x');
xlabel('Time (s)');
ylabel('x');

% Plotting the results of QP2
figure(2);
subplot(2,1,1);
plot(timeSteps, Uex_all_qp2, 'LineWidth', 2);
hold on
plot(timeSteps, Uref_all, 'LineWidth', 2);
hold off
title('QP2: Control input Uex');
xlabel('Time (s)');
ylabel('Uex');

subplot(2,1,2);
plot(timeSteps, x_all_qp2, 'LineWidth', 2);
title('QP2: State x');
xlabel('Time (s)');
ylabel('x');


% Plotting the results of pQP1
figure(3);
subplot(2,1,1);
plot(timeSteps, Uex_all_pQP1, 'LineWidth', 2);
hold on
plot(timeSteps, Uref_all, 'LineWidth', 2);
hold off
title('pQP1: Control input Uex');
xlabel('Time (s)');
ylabel('Uex');

subplot(2,1,2);
plot(timeSteps, x_all_pQP1, 'LineWidth', 2);
title('pQP1: State x');
xlabel('Time (s)');
ylabel('x');

% Plotting the results of pQP2
figure(4);
subplot(2,1,1);
plot(timeSteps, Uex_all_pQP2, 'LineWidth', 2);
hold on
plot(timeSteps, Uref_all, 'LineWidth', 2);
hold off
title('pQP2: Control input Uex');
xlabel('Time (s)');
ylabel('Uex');

subplot(2,1,2);
plot(timeSteps, x_all_pQP2, 'LineWidth', 2);
title('pQP2: State x');
xlabel('Time (s)');
ylabel('x');

% Plotting the results of QP1_ff
figure(5);
subplot(2,1,1);
plot(timeSteps, Uex_all_qp1_ff, 'LineWidth', 2);
hold on
plot(timeSteps, Uref_all, 'LineWidth', 2);
hold off
title('QP1ff: Control input Uex');
xlabel('Time (s)');
ylabel('Uex');

subplot(2,1,2);
plot(timeSteps, x_all_qp1_ff, 'LineWidth', 2);
title('QP1ff: State x');
xlabel('Time (s)');
ylabel('x');

% Plotting the results of QP2_ff
figure(6);
subplot(2,1,1);
plot(timeSteps, Uex_all_qp2_ff, 'LineWidth', 2);
hold on
plot(timeSteps, Uref_all, 'LineWidth', 2);
hold off
title('QP2ff: Control input Uex');
xlabel('Time (s)');
ylabel('Uex');

subplot(2,1,2);
plot(timeSteps, x_all_qp2_ff, 'LineWidth', 2);
title('QP2ff: State x');
xlabel('Time (s)');
ylabel('x');

% Plotting the results of pQP1_ff
figure(7);
subplot(2,1,1);
plot(timeSteps, Uex_all_pQP1_ff, 'LineWidth', 2);
hold on
plot(timeSteps, Uref_all, 'LineWidth', 2);
hold off
title('pQP1ff: Control input Uex');
xlabel('Time (s)');
ylabel('Uex');

subplot(2,1,2);
plot(timeSteps, x_all_pQP1_ff, 'LineWidth', 2);
title('pQP1ff: State x');
xlabel('Time (s)');
ylabel('x');

% Plotting the results of pQP2_ff
figure(8);
subplot(2,1,1);
plot(timeSteps, Uex_all_pQP2_ff, 'LineWidth', 2);
hold on
plot(timeSteps, Uref_all, 'LineWidth', 2);
hold off
title('pQP2ff: Control input Uex');
xlabel('Time (s)');
ylabel('Uex');

subplot(2,1,2);
plot(timeSteps, x_all_pQP2_ff, 'LineWidth', 2);
title('pQP2ff: State x');
xlabel('Time (s)');
ylabel('x');