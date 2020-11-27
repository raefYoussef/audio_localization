function [est_pos, P] = TDOA_EKF(true_range_diff, sensors1, sensors2, pos_init, P_init)
% Estimate position of speaker using EKF on measured range differences

    sigma = 1; % no info on sensor variance
    Pn = P_init;
    xn = pos_init;

    num_meas = length(true_range_diff);

    % debug
    x_hist = zeros(3, num_meas);
    % x_bounds = zeros(2, num_meas, 2);

    % loop init
    max_iter = 5000;
    est_epislon = 1e-6;
    est_diff = 1e6;
    est_old = pos_init;
    min_diff = est_diff;
    iter = 0;

    while est_diff > est_epislon && est_diff < (10*min_diff) && iter < max_iter
        for i = 1:num_meas
            est_range_diff = calc_range_diff(xn, sensors1(:,i), sensors2(:,i));

            Hn1 =  TDOA_Jacobian(xn, sensors1(:,i), sensors2(:,i));
            Rn1 = sigma.^2;
            Kn1 = Pn * Hn1.' * inv(Hn1 * Pn * Hn1.' + Rn1);
            xn1 = xn + Kn1 * (true_range_diff(i) - est_range_diff);
            Pn1 = (eye(3) - (Kn1 * Hn1)) * Pn;

            Pn = Pn1;
            xn = xn1;
            x_hist(:,i) = xn;

    %         % calc 95% bounds
    %         % one deg of freedom since we estimate one parameter at a time
    %         chi_val = chi2inv(.95,1); 
    %         x_hf_range = sqrt(chi_val * diag(Pn)); 
    %         x_bounds(:,i,1) = [xn(1) - x_hf_range(1); xn(1) + x_hf_range(1)];
    %         x_bounds(:,i,2) = [xn(2) - x_hf_range(2); xn(2) + x_hf_range(2)];
    %         x_bounds(:,i,3) = [xn(3) - x_hf_range(3); xn(3) + x_hf_range(3)];
        end
        
        % calc estimate diff 
        est_new = xn;
        est_diff = norm(est_old - est_new);
        true_diff = norm(true_range_diff - calc_range_diff(est_new, sensors1, sensors2));
        est_old = est_new;
        
        % print iter info
        iter = iter + 1;
        fprintf('Iter %d: x_hat = %f, y_hat = %f, z_hat = %f, est_diff = %.5f, meas_diff = %.5f\n',...
                 iter, est_new(1), est_new(2), est_new(3), est_diff, true_diff);   
    end

    est_pos = est_new;
    P = Pn;

end

