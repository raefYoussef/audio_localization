function [est_l1, est_l2] = TDOA_grid_search(sensor1_pos, sensor2_pos, rdoa_meas,...
                                     center, range, step,...
                                     flag_3d, flag_debug)
% Perform grid search over specified space to find best position estimate

    if flag_3d
        % calc grid points
        ndim = 3;
        pts_range = (-range/2):step:(range/2);
        z_range = 0:step:(range/2);
        [pts_x, pts_y, pts_z] = meshgrid(center(1) + pts_range, center(2) + pts_range, center(3) + z_range);
        pts = [reshape(pts_x, 1, []); reshape(pts_y, 1, []); reshape(pts_z, 1, [])];
    else
        ndim = 2;
        sensor1_pos = sensor1_pos(1:2, :);
        sensor2_pos = sensor2_pos(1:2, :);
        center = center(1:2);
        
        % calc grid points
        pts_range = -range/2:step:range/2;
        [pts_x, pts_y] = meshgrid(center(1) + pts_range, center(2) + pts_range);
        pts = [reshape(pts_x, 1, []); reshape(pts_y, 1, [])];
    end
    
    % calc TDOA for each grid point
    npts = size(pts, 2);
    est_diff_l1 = zeros(1, npts);
    est_diff_l2 = zeros(1, npts);

    for i = 1:npts
        est_tdoa = calc_range_diff(pts(:, i), sensor1_pos, sensor2_pos);
        est_diff_l1(i) = norm(rdoa_meas - est_tdoa, 1);
        est_diff_l2(i) = norm(rdoa_meas - est_tdoa, 2);
    end
    
    % view results
    if flag_debug && ndim == 2
        reshaped_diff_l1 = reshape(est_diff_l1, size(pts_x,1), []);
        reshaped_diff_l2 = reshape(est_diff_l2, size(pts_x,1), []);

        figure();
        surf(pts_x, pts_y, reshaped_diff_l1);
        title('L1 Grid Search');
        
        figure();
        surf(pts_x, pts_y, reshaped_diff_l2);
        title('L2 Grid Search');
    end
    
    % search for minimum difference estimate
    [min_val1, min_indx1] = min(est_diff_l1);
    est_l1 = pts(:, min_indx1);
    
    [min_val2, min_indx2] = min(est_diff_l2);
    est_l2 = pts(:, min_indx2);
    
end

