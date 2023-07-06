function x_updated = updateState(Ad, Bd, Wd, x, u)
    % Compute the updated state
    x_updated = Ad*x + Bd*u + Wd;
end
