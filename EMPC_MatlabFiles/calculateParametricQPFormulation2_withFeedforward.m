function solution = calculateParametricQPFormulation2_withFeedforward(Aex, Bex, Cex, Qex, Rex, Urefex, steer_lim, lateral_error_lim, N, DIM_U, DIM_X, modelType)
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
    Zex = sdpvar(DIM_U*N + DIM_X*N, 1);
    x0 = sdpvar(DIM_X, 1);

    % qp formulation 2
    % cost function: 1/2 * Zex' * H * Zex + f' * Zex, Zex = [Xex; delta_Uex]
    % H = [Cex' * Qex * Cex 0
    %       0              Rex]
    % f' = 0 
    H = zeros(DIM_X * N + DIM_U * N, DIM_X * N + DIM_U * N);
    H(1:DIM_X * N, 1:DIM_X * N) = Cex' * Qex * Cex;
    H(DIM_X * N+1:end, DIM_X * N+1:end) = Rex;
    J = 1/2*Zex'*H*Zex;
    % constraint matrix : lb < Zex < ub, lbA = A*Zex = ubA
    % A = [-I, Bex]
    % lbA = -Aex * x0 - Wex
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
    ub(DIM_X * N+1:end) = repmat(u_lim, [N, 1]) - Urefex;
    C = [lb <= Zex <= ub, A*Zex == lbA];

    % convert the problem into MPT format
    plp = Opt(C, J, x0, Zex);
    % tell MPT to construct the parametric solution
    solution = plp.solve();
end