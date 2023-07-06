function delta_Uex = calculateQPFormulation2_withFeedforward(x0, Aex, Bex, Cex, Qex, Rex, Urefex, steer_lim, lateral_error_lim, N, DIM_U, DIM_X, modelType)
    u_lim = deg2rad(steer_lim);
    x_lim = zeros(DIM_X, 1);
    x_lim(1) = lateral_error_lim; 
    switch modelType
        case 'DynamicsBicycleModel'
            x_lim(2:end) = repmat(1000, [DIM_X - 1, 1]);
        case 'DynamicsBicycleModelWithDelay'
            x_lim(2:4) = [1000;1000;1000];
            x_lim(5) = u_lim - Urefex(1);
        case 'KinematicsBicycleModel'
            x_lim(2:end) = repmat(1000, [DIM_X - 1, 1]);
        case 'KinematicsBicycleModelWithDelay'
            x_lim(2) = 1000;
            x_lim(3) = u_lim - Urefex(1);
        otherwise
            error('Unknown model type');
    end
    % qp formulation 2
    % cost function: 1/2 * Zex' * H * Zex + f' * Zex, Zex = [Xex; delta_Uex]
    % H = [Cex' * Qex * Cex 0
    %       0              Rex]
    % f' = 0
    H = zeros(DIM_X * N + DIM_U * N, DIM_X * N + DIM_U * N);
    H(1:DIM_X * N, 1:DIM_X * N) = Cex' * Qex * Cex;
    H(DIM_X * N+1:end, DIM_X * N+1:end) = Rex;
    f = zeros(1,DIM_X*N + DIM_U*N);
    % constraint matrix : lb < Zex < ub, lbA = A*Zex = ubA
    % A = [-I, Bex]
    % lbA = -Aex * x0
    % ubA = lbA
    % lb = [-xlim; -xlim; ... ; -xlim; -delta_u_lim; -delta_u_lim; ... ; -delta_u_lim]
    % ub = [xlim; xlim; ... ; xlim; delta_u_lim; delta_u_lim; ... ; delta_u_lim]
    A = zeros(DIM_X * N, DIM_X * N + DIM_U * N);
    A(1:DIM_X * N, 1:DIM_X * N) = eye(DIM_X * N);
    A(1:DIM_X * N, DIM_X * N+1:end) = -Bex;

    lbA = Aex * x0;
    ubA = lbA;

    lb = zeros(DIM_X * N + DIM_U * N, 1);
    ub = zeros(DIM_X * N + DIM_U * N, 1);
    lb(1:DIM_X * N) = repmat(-x_lim, [N, 1]);
    ub(1:DIM_X * N) = repmat(x_lim, [N, 1]);
    lb(DIM_X * N+1:end) = repmat(-u_lim, [N, 1]) - Urefex;
    ub(DIM_X * N+1:end) = repmat(u_lim, [N, 1]) - Urefex ;

    options = optimoptions('quadprog', 'Display', 'off');
    start = tic;
    Zex = quadprog(H, f, [], [], A, lbA, lb, ub, [], options);
    elapsed = toc(start) * 1000; % milliseconds
    fprintf('[MPC] calculateMPC: qp solver calculation time = %.2f [ms]\n', elapsed);

    if any(isnan(Zex))
       error('[MPC] calculateMPC: model Zex includes NaN, stop MPC.');
    end
       delta_Uex = Zex(DIM_X * N + 1:end);
end
