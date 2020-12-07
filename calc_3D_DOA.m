function [centers, doa] = calc_3D_DOA(mic_arr, Fs, sensor_pos, sensor_indices, flag_debug)
% Given eight sensors centered around point, computer DOA for each
% center. 
% Note: - Format
% [s1; s2; s3; s4; s5; s6; s7; s8]
% Where two consecutive sensors are across each other, each consecutive
% four sensors form an othogonal pairs, and each columns correspond to 
% sensors across the same center 

    % Constants
    SPD_OF_SOUND = 343; % m/s
%     true_pos = [0.063; 0.063; 10];
    npairs = 4;
    ncenters = size(sensor_indices,2);
    max_rdoa = zeros(npairs, ncenters);
    rdoa = zeros(npairs, ncenters);
    
    for i = 1:npairs
        % calc max range diff
        max_rdoa(i,:) = vec_mag(sensor_pos(:, sensor_indices(2*i-1, :)) - sensor_pos(:, sensor_indices(2*i, :)));
    
        % calc TDOA/Range Diff
        [tdoa, ~] = calc_TDOA(mic_arr, Fs, sensor_pos, sensor_indices([2*i-1 2*i], :));
        calc_rdoa = tdoa.' * SPD_OF_SOUND;
        
%         calc_rdoa = calc_range_diff(true_pos,...
%                                     sensor_pos(:,sensor_indices(2*i-1,:)),... 
%                                     sensor_pos(:,sensor_indices(2*i,:))).';
        
        % handle sampling issues
        calc_rdoa(abs(calc_rdoa) > max_rdoa(i,:)) = sign(calc_rdoa(abs(calc_rdoa) > max_rdoa(i,:))) .* max_rdoa(i, abs(calc_rdoa) > max_rdoa(i,:));
        rdoa(i,:) = calc_rdoa;
    end
    
    % normalize RDOA
    norm_rdoa = rdoa ./ max_rdoa;
    
    % sort RDOA
    [~, sorted_indx] = sort(abs(norm_rdoa), 1);
        
    % estimate phi from axis best aligned with target
    phi = zeros(1,ncenters);
    
    for i = 1:ncenters
        % determine axis orthogonal to target (least RDOA)
        % axis orthogonal to that is best aligned with target
        switch sorted_indx(1,i)
            case 1
                phi(i) = asind(norm_rdoa(2,i));
            case 2
                phi(i) = asind(norm_rdoa(1,i));
            case 3
                phi(i) = asind(norm_rdoa(4,i));
            case 4
                phi(i) = asind(norm_rdoa(3,i));
        end
    end
    
    % readjust norms based on phi angle
    norm_rdoa = rdoa ./ (sind(phi) .* max_rdoa);
    % adjust for a phi = 90 i.e. div by 0
    norm_rdoa(isnan(norm_rdoa)) = 0;
    
    % calc centers
    local_axis = (sensor_pos(:, sensor_indices(2, :)) - sensor_pos(:, sensor_indices(1, :)));
    centers = (.5*local_axis) + sensor_pos(:, sensor_indices(1, :));
    
    % calculate angle difference between local and global axis (east = 0 deg)
    local_offset = zeros(npairs, ncenters);
    
    for i = 1:npairs
        local_axis = (sensor_pos(:, sensor_indices(2*i, :)) - sensor_pos(:, sensor_indices(2*i-1, :)));
        local_offset(i,:) = atan2d(local_axis(2,:), local_axis(1,:));
    end
    
    % calculate angle with respect to sensor axis
    local_angle = acosd(norm_rdoa); 
    
    local_angle1 = zeros(2, ncenters);
    local_angle2 = zeros(2, ncenters);
    local_offset1 = zeros(1, ncenters);
    local_offset2 = zeros(1, ncenters);
    
    % account for symmetry, the source can be on either side of the axis
    for i = 1:ncenters
        local_angle1(:,i) = local_angle(sorted_indx(1,i),i) .* [1; -1];
        local_angle2(:,i) = local_angle(sorted_indx(2,i),i) .* [1; -1];
        
        local_offset1(i) = local_offset(sorted_indx(1,i),i);
        local_offset2(i) = local_offset(sorted_indx(2,i),i);
    end
    
    % calculate global angles
    global_angle1 = mod(local_angle1 + local_offset1, 360);
    global_angle2 = mod(local_angle2 + local_offset2, 360);

    % disambiguate the true angle
    angle_diff = abs([(global_angle1(1,:) - global_angle2); (global_angle1(2,:) - global_angle2)]);
    min_diff = angle_diff == min(angle_diff);
    
    theta = zeros(1, ncenters);

    for i = 1:ncenters
        min_indx = find(min_diff(:,i), 1);
        
        if min_indx == 1
            theta(i) = mean([global_angle1(1,i), global_angle2(1,i)]);
        elseif min_indx == 2
            theta(i) = mean([global_angle1(1,i), global_angle2(2,i)]);
        elseif min_indx == 3
            theta(i) = mean([global_angle1(2,i), global_angle2(1,i)]);
        elseif min_indx == 4
            theta(i) = mean([global_angle1(2,i), global_angle2(2,i)]);
        end
    end
    
    doa = [theta; phi];
    
    % debugging
    if flag_debug
        rho = 5;
        figure();
        
        for i = 1:ncenters
            start = centers(:,i);
            delta = [rho.*cosd(theta(i)).*sind(phi(i)); rho.*sind(theta(i)).*sind(phi(i)); rho.*cosd(phi(i))];
            stop = start + delta;
            plot3([start(1); stop(1)], [start(2); stop(2)], [start(3); stop(3)], '--');
            hold on;
        end
    end
end


