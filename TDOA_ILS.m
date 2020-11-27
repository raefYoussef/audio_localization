function [est_pos, P, conv_flag] = TDOA_ILS(true_range_diff, sensors1, sensors2, pos_init, flag_3d, flag_debug)
% Estimate position of speaker using ILS on measured range differences

    % ILS
    num_meas = length(true_range_diff);
    R = eye(num_meas); % no info on measurement variance
    
    % handle 2D case
    if ~flag_3d 
        sensors1 = sensors1(1:2, :);
        sensors2 = sensors2(1:2, :);
        pos_init = pos_init(1:2);
    end
    
    % loop init
    max_iter = 50;
    est_epislon = 1e-6;
    est_old = pos_init;
    est_diff = 1e6;
    min_diff = est_diff;
    iter = 0;

    while est_diff > est_epislon && est_diff < (10*min_diff) && iter < max_iter
        % calc dist based on curr estimate
        est_range_diff = calc_range_diff(est_old,...
                                        sensors1,...
                                        sensors2);
        % calc jacobian
        H = TDOA_Jacobian(est_old,...
                          sensors1,...
                          sensors2);

        % calc new estimate
        est_new = est_old + inv((H.')*inv(R)*H)*(H.')*inv(R)*(true_range_diff - est_range_diff);

        % calc estimate diff 
        est_diff = norm(est_old - est_new);
        meas_diff = norm(true_range_diff - calc_range_diff(est_new, sensors1, sensors2));
        est_old = est_new;

        if est_diff < min_diff
            min_diff = est_diff;
        end

        % print iter info
        iter = iter + 1;

        if flag_debug
            if flag_3d
                fprintf('Iter %d: x_hat = %f, y_hat = %f, z_hat = %f, err_diff = %.4f, meas_diff = %.4f\n',...
                        iter, est_new(1), est_new(2), est_new(3), est_diff, meas_diff);    
            else
                fprintf('Iter %d: x_hat = %f, y_hat = %f, err_diff = %.4f, meas_diff = %.4f\n',...
                iter, est_new(1), est_new(2), est_diff, meas_diff);    
            end
        end
    end

    est_pos = est_new;                      % estimated center
    P = inv(H.' * inv(R) * H);              % covariance matrix
    conv_flag = est_diff < (10*min_diff);   % convergence flag

end

