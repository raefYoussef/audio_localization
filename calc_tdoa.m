function [tdoa_lags, tdoa_corr] = calc_tdoa(mic_arr, Fs, sensors1, sensors2, max_lags)
% Calculate time difference of arrival between each pair of microphones
% Format is t_sensor1 - t_sensor2
% TODO: add upsampling?

    Ts = 1/Fs;
    narray = size(mic_arr,2);
    nmeas = (narray*(narray-1))/2;
    tdoa_lags = zeros(nmeas,1);
    tdoa_corr = zeros(nmeas,1);
    
    for i = 1:nmeas
        sensor1 = sensors1(i);
        sensor2 = sensors2(i);
        max_lag = max_lags(i);
        
        [c, lags] = xcorr(mic_arr(:,sensor1), mic_arr(:,sensor2), max_lag);
        
        mx_corr = max(c, [], 'all');
        mx_lag = lags(c == mx_corr);
        tdoa_corr(i) = mx_corr;
        tdoa_lags(i) = mean(mx_lag); % TODO: fix multiple peaks, upsampling??
    end
    
    tdoa_lags = Ts * tdoa_lags;
end

