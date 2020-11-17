function magn = vec_mag(vector)
% Calculate vector magnitude 
magn = sqrt(diag((vector).' * (vector))).';
end

