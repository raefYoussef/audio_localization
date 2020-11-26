function flag = detect_activity(mics, threshold)
% Detect activity based on channel amplitude

flag = all(max(mics) > threshold);

end

