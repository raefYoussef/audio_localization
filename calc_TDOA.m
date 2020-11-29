function [tdoa_lags, tdoa_corr] = calc_TDOA(mic_arr, Fs, sensor_pos, sensor_indices)
% Calculate time difference of arrival between each pair of microphones
% Format is t_sensor1 - t_sensor2

    Ts = 1/Fs;
    nmeas = size(sensor_indices,2);
    tdoa_lags = zeros(nmeas,1);
    tdoa_corr = zeros(nmeas,1);
    sensors1_indx = sensor_indices(1, :);
    sensors2_indx = sensor_indices(2, :);
    
    % calc max lags
    max_lags = calc_max_lag(sensor_pos(:, sensors1_indx), sensor_pos(:, sensors2_indx), Fs);
    
    for i = 1:nmeas
        sensor1 = sensors1_indx(i);
        sensor2 = sensors2_indx(i);
        max_lag = max_lags(i);
        
        [c, lags] = xcorr(mic_arr(:,sensor1), mic_arr(:,sensor2), max_lag);
        
        mx_corr = max(c, [], 'all');
        mx_lag = lags(c == mx_corr);
        tdoa_corr(i) = mx_corr;
        tdoa_lags(i) = mean(mx_lag); % TODO: fix multiple peaks, upsampling??
    end
    
    tdoa_lags = Ts * tdoa_lags;
end

