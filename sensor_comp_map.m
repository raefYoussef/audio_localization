function [sensor1,sensor2] = sensor_comp_map(nsensors)
% Return a map of which sensor contribute to each tdoa measurements

cnt = 1;
nmeas = (nsensors*(nsensors-1))/2;
sensor1 = zeros(1,nmeas);
sensor2 = zeros(1,nmeas);

for i = 1:nsensors
    for j = i:nsensors
        if i ~= j
            sensor1(cnt) = i;
            sensor2(cnt) = j;
            cnt = cnt + 1;
        end
    end
end
end

