function result = TDOA_Jacobian(target, sensor1, sensor2)
% Calculate TDOA Jacobian 
result = (unit_vec(target-sensor1) - unit_vec(target-sensor2)).';
end

