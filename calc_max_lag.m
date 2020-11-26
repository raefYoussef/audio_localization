function max_lag = calc_max_lag(sensor1_pos, sensor2_pos, Fs)
% Calculate max samples lag based on geometry

SPD_OF_SOUND = 343; % m/s
ERR_MARGIN = 1.25;  % 25% error margin

sensor_dist = vec_mag(sensor1_pos - sensor2_pos);
max_lag = ceil(sensor_dist ./ (SPD_OF_SOUND/Fs) * ERR_MARGIN);

end

