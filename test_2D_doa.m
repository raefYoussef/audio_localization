clear;
clc;
close all;

%% 2D Angle Sweep
indx = 0:1:180;
r = 2;
targets = r .* [cosd(indx); sind(indx)];
ntargets = size(targets,2);

%% Compute RDOA
sensors = [1 0; -1 0].';
center = mean(sensors,2);
max_dist = vec_mag(sensors(:,2) - sensors(:,1));

rdoa = zeros(1, ntargets);
true_theta = zeros(1, ntargets);
est_theta = zeros(1, ntargets);

for i = 1:ntargets
    rdoa(i) = calc_range_diff(targets(:,i), sensors(:,2), sensors(:,1));
    est_theta(i) = acosd(rdoa(i) ./ max_dist);
    diff = targets(:,i) - center;
    true_theta(i) = atan2d(diff(2), diff(1));
end

%% Visualization
error = true_theta - est_theta;
figure();
plot(indx, error);
title('DOA Estimate Error, Distance = ' + string(r) + ' meters');
xlabel('Target True DOA (Deg)');
ylabel('DOA Error (Deg)');

figure();
plot(indx, rdoa ./ max_dist);
hold on;
plot(indx, cosd(true_theta));