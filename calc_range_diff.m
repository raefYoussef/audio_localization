function result = calc_range_diff(target, sensor1, sensor2)
% Calculate range difference for a given target and two sensors 
result = (vec_mag(target-sensor1) - vec_mag(target-sensor2)).';
end

