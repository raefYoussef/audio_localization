function [center, doa] = calc_DOA(mic_arr, Fs, sensor_pos, sensor_indices)
% Given three sensors audio, compute two centers and
% their DOA
% Note: - the three sensors have to form two orthogonal vectors. 
%       - The first index is the corner of the three sensors

    % Constants
    SPD_OF_SOUND = 343; % m/s

    % we currently can only handle 2D angles
    sensor_pos = sensor_pos(1:2,:);

    % calc max range diff
    max_rdoa1 = vec_mag(sensor_pos(:, sensor_indices(1, :)) - sensor_pos(:, sensor_indices(2, :)));
    max_rdoa2 = vec_mag(sensor_pos(:, sensor_indices(1, :)) - sensor_pos(:, sensor_indices(3, :)));
    
    % calc TDOA/Range Diff
    [tdoa1, ~] = calc_TDOA(mic_arr, Fs, sensor_pos, sensor_indices([1 2], :));
    [tdoa2, ~] = calc_TDOA(mic_arr, Fs, sensor_pos, sensor_indices([1 3], :));
    rdoa1 = tdoa1.' * SPD_OF_SOUND;
    rdoa2 = tdoa2.' * SPD_OF_SOUND;

    % handle sampling issues
    rdoa1(abs(rdoa1) > max_rdoa1) = sign(rdoa1(abs(rdoa1) > max_rdoa1)) .* max_rdoa1(abs(rdoa1) > max_rdoa1);
    rdoa2(abs(rdoa2) > max_rdoa2) = sign(rdoa2(abs(rdoa2) > max_rdoa2)) .* max_rdoa2(abs(rdoa2) > max_rdoa2);

    % calculate angle with respect to sensor axis
    local_angle1 = acosd(rdoa1 ./ max_rdoa1); 
    local_angle2 = acosd(rdoa2 ./ max_rdoa2);

    % account for symmetry, the source can be on either side of the axis
    local_angle1 = local_angle1 .* [1; -1];
    local_angle2 = local_angle2 .* [1; -1];

    % calculate angle difference between local and global axis (east = 0 deg)
    local_axis1 = (sensor_pos(:, sensor_indices(2, :)) - sensor_pos(:, sensor_indices(1, :)));
    local_axis2 = (sensor_pos(:, sensor_indices(3, :)) - sensor_pos(:, sensor_indices(1, :)));
    local_offset1 = mod(atan2d(local_axis1(2,:), local_axis1(1,:)), 360);
    local_offset2 = mod(atan2d(local_axis2(2,:), local_axis2(1,:)), 360);
    
    % calculate global angles
    global_angle1 = mod(local_angle1 + local_offset1, 360);
    global_angle2 = mod(local_angle2 + local_offset2, 360);

    % disambiguate the true angle
    % assumption: source is outside microphone array and it's far
    % TODO: handle a source inside the array or close to it

    angle_diff = abs([(global_angle1(1,:) - global_angle2); (global_angle1(2,:) - global_angle2)]);
    min_diff = angle_diff == min(angle_diff);
    
    nmeas = size(angle_diff,2);
    center = zeros(2, 2*nmeas);
    doa = zeros(1, 2*nmeas);

    for i = 1:nmeas
        min_indx = find(min_diff(:,i), 1);
        
        if min_indx == 1
            doa(2*i-1) = global_angle1(1,i);
            doa(2*i)   = global_angle2(1,i);
        elseif min_indx == 2
            doa(2*i-1) = global_angle1(1,i);
            doa(2*i)   = global_angle2(2,i);
        elseif min_indx == 3
            doa(2*i-1) = global_angle1(2,i);
            doa(2*i)   = global_angle2(1,i);
        elseif min_indx == 4
            doa(2*i-1) = global_angle1(2,i);
            doa(2*i)   = global_angle2(2,i);
        end

        center(:,2*i-1) = local_axis1(:,i) / 2 + sensor_pos(:, sensor_indices(1, i));
        center(:,2*i)   = local_axis2(:,i) / 2 + sensor_pos(:, sensor_indices(1, i));
    end

end


