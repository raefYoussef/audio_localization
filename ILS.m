function [est_pos, P] = ILS(true_range_diff, sensors1, sensors2, pos_init)
% Estimate position of speaker using ILS on measured range differences

    % ILS
    num_meas = length(true_range_diff);
    R = eye(num_meas); % no info on measurement variance
    
    % loop init
    max_iter = 50;
    est_epislon = 1e-6;
    est_old = pos_init;
    est_diff = 1e6;
    min_diff = est_diff;
    diff_hist = zeros(1,max_iter);
    iter = 0;

while est_diff > est_epislon && est_diff < (10*min_diff) && iter < max_iter
    % calc dist based on curr estimate
    est_range_diff = calc_range_diff(repmat(est_old, 1, num_meas),...
                                    sensors1,...
                                    sensors2);
    % calc jacobian
    H = Jacobian(repmat(est_old, 1, num_meas),...
                 sensors1,...
                 sensors2);
             
    % calc new estimate
    est_new = est_old + inv((H.')*inv(R)*H)*(H.')*inv(R)*(true_range_diff - est_range_diff);
    
    % calc estimate diff 
    est_diff = norm(est_old - est_new);
    est_old = est_new;
    % print iter info
    iter = iter + 1;
%     fprintf('Iter %d: x_hat = %f, y_hat = %f, z_hat = %f,\n',...
%             iter, est_new(1), est_new(2), est_new(3));    
end

est_pos = est_new;         % estimated center
P = inv(H.' * inv(R) * H); % covariance matrix

end

