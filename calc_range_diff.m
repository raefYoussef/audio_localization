function result = calc_range_diff(target, sensor1, sensor2)
% Calculate range difference for a given target and two sensors 
result = (magn(target-sensor1) - magn(target-sensor2)).';
end

