function delta_Uex = calculateQPFormulation1_withFeedforward(x0, Aex, Bex, Cex, Qex, Rex, Urefex, steer_lim, lateral_error_lim, N, DIM_U, DIM_X, modelType)
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
    % qp formulation 1
    % cost function: 1/2 * Uex' * H * Uex + f' * delta_Uex
    % H = (Cex * Bex)' * Qex * (Cex * Bex) + Rex
    % f' = (Cex * Aex * x0)' * Qex * (Cex * Bex)
    CB = Cex * Bex;
    QCB = Qex * CB;
    H = CB' * QCB + Rex;
    H = (H + H') / 2;  % making H symmetric
    f = (Cex * Aex * x0)' * QCB ;
    % constraint matrix : lb < delta_Uex < ub, lbA < A*delta_Uex <ubA
    % lb = [-delta_u_lim; -delta_u_lim; ... ; -delta_u_lim]
    % ub = [delta_u_lim; delta_u_lim; ... ; delta_u_lim]
    % A = Bex
    % lbA = [-x_lim; -x_lim; ... ; -x_lim] - Aex * x0
    % ubA = [x_lim; x_lim; ... ; x_lim] - Aex * x0
    lb = repmat(-u_lim, [N, 1]) - Urefex; % applying min steering limit
    ub = repmat(u_lim, [N, 1]) - Urefex;  % applying max steering limit
    A = Bex;
    lbA = repmat(-x_lim, [N, 1]) - Aex * x0; % apllying  min lateral deviation limit
    ubA = repmat(x_lim, [N, 1]) - Aex * x0;  % apllying max lateral deviation limit

    start = tic;
    % Initialize options for quadprog
    options = optimoptions('quadprog', 'Display', 'off');
    % Modify the quadprog call
    [delta_Uex, fval, exitflag, output] = quadprog(H, f, A, ubA, [], [], lb, ub, [], options);
    elapsed = toc(start) * 1000; % milliseconds
    fprintf('[MPC] calculateMPC: qp solver calculation time = %.2f [ms]\n', elapsed);

    if exitflag ~= 1
        error('[MPC] qp solver error');
    end

    if any(isnan(delta_Uex))
        error('[MPC] calculateMPC: model Uex includes NaN, stop MPC.');
    end
end