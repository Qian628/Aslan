function solution = calculateParametricQPFormulation1(Aex, Bex, Cex, Qex, Rex, Wex, Urefex, steer_lim, lateral_error_lim, N, DIM_U, DIM_X, modelType)
    u_lim = deg2rad(steer_lim);
    x_lim = zeros(DIM_X, 1);
    x_lim(1) = lateral_error_lim; 
    switch modelType
        case 'DynamicsBicycleModel'
            x_lim(2:end) = repmat(1000, [DIM_X - 1, 1]);
        case 'DynamicsBicycleModelWithDelay'
            x_lim(2:4) = [1000;1000;1000];
            x_lim(5) = u_lim;
        case 'KinematicsBicycleModel'
            x_lim(2:end) = repmat(1000, [DIM_X - 1, 1]);
        case 'KinematicsBicycleModelWithDelay'
            x_lim(2) = 1000;
            x_lim(3) = u_lim;
        otherwise
            error('Unknown model type');
    end
    Uex = sdpvar(DIM_U*N, 1);
    x0 = sdpvar(DIM_X, 1);
    % qp formulation 1
    % cost function: 1/2 * Uex' * H * Uex + f' * Uex
    % H = (Cex * Bex)' * Qex * (Cex * Bex) + Rex
    % f' = (Cex * (Aex * x0 + Wex))' * Qex * (Cex * Bex) - Urefex' * Rex
    CB = Cex * Bex;
    QCB = Qex * CB;
    H = CB' * QCB + Rex;
    H = (H + H') / 2;  % making H symmetric
    f = (Cex * (Aex * x0 + Wex))' * QCB - Urefex' * Rex;
    J = 1/2*Uex'*H*Uex + f*Uex;
    % constraint matrix : lb < Uex < ub, lbA < A*Uex <ubA
    % lb = [-u_lim; -u_lim; ... ; -u_lim]
    % ub = [u_lim; u_lim; ... ; u_lim]
    % A = Bex
    % lbA = [-x_lim; -x_lim; ... ; -x_lim] - Aex * x0 - Wex
    % ubA = [x_lim; x_lim; ... ; x_lim] - Aex * x0 - Wex 
    lb = repmat(-u_lim, [N, 1]); % applying min steering limit
    ub = repmat(u_lim, [N, 1]);  % applying max steering limit
    A = Bex;
    lbA = repmat(-x_lim, [N, 1]) - Aex * x0 - Wex; % apllying  min lateral deviation limit
    ubA = repmat(x_lim, [N, 1]) - Aex * x0 - Wex;  % apllying max lateral deviation limit
    C = [lb <= Uex <= ub, lbA <= A*Uex <= ubA];
    % convert the problem into MPT format
    plp = Opt(C, J, x0, Uex);
    % tell MPT to construct the parametric solution
    solution = plp.solve();
end