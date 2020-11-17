function [tdoa_lags, tdoa_corr] = calc_tdoa(mic_arr, Fs)
% Calculate time difference of arrival between each pair of microphones
% TODO: add upsampling?

    Ts = 1/Fs;
    narray = size(mic_arr,2);
%     tdoa_lags = zeros(narray);
%     tdoa_corr = zeros(narray);
    tdoa_lags = zeros(1,(narray*(narray-1))/2);
    tdoa_corr = zeros(1,(narray*(narray-1))/2);
    
    cnt = 1;
    for i = 1:narray
        for j = 1:i
            [c, lags] = xcorr(mic_arr(:,j), mic_arr(:,i));
            max_corr = max(c, [], 'all');
            max_lag = lags(c == max_corr);
%             tdoa_corr(i,j) = max_corr;
%             tdoa_lags(i,j) = mean(max_lag); % TODO: fix multiple peaks, upsampling??
            if i ~= j
                tdoa_corr(cnt) = max_corr;
                tdoa_lags(cnt) = mean(max_lag); % TODO: fix multiple peaks, upsampling??
                cnt = cnt + 1;
            end
        end
    end
    
    tdoa_lags = Ts*tdoa_lags;
end

