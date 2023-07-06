function [Aex, Bex, Wex, Cex, Qex, Rex, Urefex] = calculateMPCMatrix(mpc_resampled_ref_traj, mpc_param_, modelType, vehicle_model_ptr_, Q, R, N, DT, DIM_X, DIM_U, DIM_Y)
    
    Aex = zeros(N*DIM_X, DIM_X);
    Bex = zeros(N*DIM_X, N*DIM_U);
    Wex = zeros(N*DIM_X, 1);
    Cex = zeros(N*DIM_Y, N*DIM_X);
    Qex = zeros(N*DIM_Y, N*DIM_Y);
    Rex = zeros(N*DIM_U, N*DIM_U);
    Urefex = zeros(N*DIM_U, 1);
    
    mpc_curr_time = 0;

    for i = 1:N
        ref_k = mpc_resampled_ref_traj.k(i);
        ref_vx = mpc_resampled_ref_traj.vx(i);
        ref_vx_squared = ref_vx^2;

        [Ad, Bd, Cd, Wd] = vehicle_model_ptr_.calculateDiscreteMatrix(ref_vx, ref_k, DT);

        Q_adaptive = Q;
        R_adaptive = R;
        
        if i == N
            Q_adaptive(1, 1) = mpc_param_.weight_terminal_lat_error;
            Q_adaptive(2, 2) = mpc_param_.weight_terminal_heading_error;
        end
        Q_adaptive(2, 2) = Q_adaptive(2, 2) + ref_vx_squared * mpc_param_.weight_heading_error_squared_vel_coeff;
        R_adaptive(1, 1) = R_adaptive(1, 1) + ref_vx_squared * mpc_param_.weight_steering_input_squared_vel_coeff;

        idx_x_i = (i-1) * DIM_X + 1;
        idx_x_i_prev = (i-2) * DIM_X + 1;
        idx_u_i = (i-1) * DIM_U + 1;
        idx_y_i = (i-1) * DIM_Y + 1;
        
        if i == 1
            Aex(1:DIM_X, 1:DIM_X) = Ad;
            Bex(1:DIM_X, 1:DIM_U) = Bd;
            Wex(1:DIM_X, 1) = Wd;
        else
            Aex(idx_x_i:idx_x_i+DIM_X-1, 1:DIM_X) = Ad * Aex(idx_x_i_prev:idx_x_i_prev+DIM_X-1, 1:DIM_X);
            for j = 1:i-1
                idx_u_j = (j-1) * DIM_U + 1;
                Bex(idx_x_i:idx_x_i+DIM_X-1, idx_u_j:idx_u_j+DIM_U-1) = Ad * Bex(idx_x_i_prev:idx_x_i_prev+DIM_X-1, idx_u_j:idx_u_j+DIM_U-1);
            end
            Wex(idx_x_i:idx_x_i+DIM_X-1, 1) = Ad * Wex(idx_x_i_prev:idx_x_i_prev+DIM_X-1, 1) + Wd;
        end
        
        Bex(idx_x_i:idx_x_i+DIM_X-1, idx_u_i:idx_u_i+DIM_U-1) = Bd;
        Cex(idx_y_i:idx_y_i+DIM_Y-1, idx_x_i:idx_x_i+DIM_X-1) = Cd;
        Qex(idx_y_i:idx_y_i+DIM_Y-1, idx_y_i:idx_y_i+DIM_Y-1) = Q_adaptive;
        Rex(idx_u_i:idx_u_i+DIM_U-1, idx_u_i:idx_u_i+DIM_U-1) = R_adaptive;

        if strcmp(modelType, 'DynamicsBicycleModel') || strcmp(modelType, 'DynamicsBicycleModelWithDelay')
            Uref = vehicle_model_ptr_.calculateReferenceInput(ref_vx, ref_k);
        else
            Uref = vehicle_model_ptr_.calculateReferenceInput(ref_k);
        end

        if abs(Uref(1, 1)) < deg2rad(mpc_param_.zero_ff_steer_deg)
            Uref(1, 1) = 0.0; % ignore curvature noise to prevent small steering adjustments when the steering angle is zero
        end

        Urefex((i-1)*DIM_U+1:i*DIM_U, 1) = Uref;

        mpc_curr_time = mpc_curr_time + DT;
    end

    for i = 1:N-1
        v = mpc_resampled_ref_traj.vx(i);
        lateral_jerk_weight = v * v * mpc_param_.weight_lat_jerk;
        Rex((i-1)*DIM_U+1:i*DIM_U, (i-1)*DIM_U+1:i*DIM_U) = Rex((i-1)*DIM_U+1:i*DIM_U, (i-1)*DIM_U+1:i*DIM_U) + lateral_jerk_weight;
        Rex(i*DIM_U+1:(i+1)*DIM_U, (i-1)*DIM_U+1:i*DIM_U) = Rex(i*DIM_U+1:(i+1)*DIM_U, (i-1)*DIM_U+1:i*DIM_U) - lateral_jerk_weight;
        Rex((i-1)*DIM_U+1:i*DIM_U, i*DIM_U+1:(i+1)*DIM_U) = Rex((i-1)*DIM_U+1:i*DIM_U, i*DIM_U+1:(i+1)*DIM_U) - lateral_jerk_weight;
        Rex(i*DIM_U+1:(i+1)*DIM_U, i*DIM_U+1:(i+1)*DIM_U) = Rex(i*DIM_U+1:(i+1)*DIM_U, i*DIM_U+1:(i+1)*DIM_U) + lateral_jerk_weight;
    end

    if any(isnan(Aex(:))) || any(isnan(Bex(:))) || any(isnan(Cex(:))) || any(isnan(Wex(:)))
        error('calculateMPC: model matrix includes NaN, stop MPC.')
    end

end

