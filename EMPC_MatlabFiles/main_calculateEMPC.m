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

% Additional Parameters for calculateMPC function
mpc_param_.N = 70;  % number of time steps
mpc_param_.weight_lat_error = 0.1;
mpc_param_.weight_heading_error = 0;
mpc_param_.weight_steering_input = 1;
mpc_param_.weight_terminal_lat_error = 0.1;
mpc_param_.weight_terminal_heading_error = 0;
mpc_param_.weight_heading_error_squared_vel_coeff = 0;
mpc_param_.weight_steering_input_squared_vel_coeff = 0;
mpc_param_.weight_lat_jerk = 0;
mpc_param_.zero_ff_steer_deg = 2; % deg

% define operating condition
operat_cond_.curvature = 0.01; % 1/m
operat_cond_.velocity = 3; % m/s

% time step
dt = 0.1; % s

calculateEMPC(modelType, model_param_, mpc_param_, operat_cond_, dt);