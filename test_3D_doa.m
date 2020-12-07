clc;
clear;
close all;

%% Init
true_pos = [3; 1; 1]
sensors = [0, 0, 0; 1 0 0; 0 1 0; 1 1 0;].';

%% Moore Penrose Test
diff = true_pos - sensors;

rho = sqrt(sum(diff.^2,1));
theta = atan2d(diff(2,:),diff(1,:));
phi = atan2d(sqrt(diag(diff(1:2,:).' * diff(1:2,:))).', diff(3,:));

% % recompute cartesian coordinates for sanity check
% diff_recon = [rho.*cosd(theta).*sind(phi); rho.*sind(theta).*sind(phi); rho.*cosd(phi)]

est_pos = Moore_Penrose(sensors, [theta; phi])

%% Polar Test (Theta)
npts = 361;
rho = repmat(10, 1, npts);
theta = 0:1:(npts-1);
phi = repmat(30, 1, npts);

targets = [rho.*cosd(theta).*sind(phi); rho.*sind(theta).*sind(phi); rho.*cosd(phi)];
ntargets = size(targets, 2);

sensors = .5*[-1, -1, 0; 1 -1 0; -1 1 0; 1 1 0;].';
rdoa12 = zeros(size(phi));
rdoa13 = zeros(size(phi));
rdoa14 = zeros(size(phi));

for i = 1:ntargets
    rdoa12(i) = calc_range_diff(targets(:,i), sensors(:,1), sensors(:,2));
    rdoa13(i) = calc_range_diff(targets(:,i), sensors(:,1), sensors(:,3));
    rdoa14(i) = calc_range_diff(targets(:,i), sensors(:,1), sensors(:,4));
end

% visualization
figure();
plot3(theta, rdoa12, rdoa13);

figure();
plot(theta, rdoa12);
hold on;
plot(theta, rdoa13);
plot(theta, rdoa14./sqrt(2));


%% Azimuth Test (Phi)
npts = 361;
rho   = repmat(10, 1, npts);
theta = repmat(0, 1, npts);
phi   = 0:1:(npts-1);

targets = [rho.*cosd(theta).*sind(phi); rho.*sind(theta).*sind(phi); rho.*cosd(phi)];
ntargets = size(targets, 2);

sensors = .5*[-1, -1, 0; 1 -1 0; -1 1 0; 1 1 0;].';
rdoa12 = zeros(size(phi));
rdoa13 = zeros(size(phi));
rdoa14 = zeros(size(phi));
rdoa34 = zeros(size(phi));

for i = 1:ntargets
    rdoa12(i) = calc_range_diff(targets(:,i), sensors(:,1), sensors(:,2));
    rdoa13(i) = calc_range_diff(targets(:,i), sensors(:,1), sensors(:,3));
    rdoa14(i) = calc_range_diff(targets(:,i), sensors(:,1), sensors(:,4));
end

% visualization
figure();
plot3(phi, rdoa12, rdoa13);

figure();
plot(phi, rdoa12);
hold on;
plot(phi, rdoa13);
plot(phi, rdoa14);

%% Measure Theta & Phi Test
sensor1 = .5*[-1 -1 0; -1 0 0;  1 -1 0; 0 -1 0].';
sensor2 = .5*[ 1  1 0;  1 0 0; -1  1 0; 0  1 0].';
max_dist = vec_mag(sensor1 - sensor2);
local_axis = sensor1 - sensor2;
axis_bias = atan2d(local_axis(2,:), local_axis(1,:));

npts  = 9;
rho   = repmat(3, 1, npts);
theta = 0:(45/2):180;
phi   = 90:-(45/2):-90;
targets = [rho.*cosd(theta).*sind(phi); rho.*sind(theta).*sind(phi); rho.*cosd(phi)];

rdoa = zeros(4, npts);
norm_rdoa = zeros(4, npts);

for i = 1:npts
    for j = 1:size(sensor1,2)
        rdoa(j,i) = calc_range_diff(targets(:,i), sensor1(:,j), sensor2(:,j));
        norm_rdoa(j,i) = rdoa(j,i) / max_dist(j);
    end
end

phi_est = zeros(1, npts);
theta_est = zeros(1, npts);

for i = 1:npts
   [~, sorted_indx] = sort(abs(norm_rdoa(:,i)), 1); 
   min1_indx = sorted_indx(1);
   min2_indx = sorted_indx(2);
   
    % estimate phi
    switch min1_indx
        case 1
            phi_est(i) = asind(norm_rdoa(3,i));
        case 2
            phi_est(i) = asind(norm_rdoa(4,i));
        case 3
            phi_est(i) = asind(norm_rdoa(1,i));
        case 4
            phi_est(i) = asind(norm_rdoa(2,i));
    end
    
    theta_candiates1 = mod(acosd(norm_rdoa(min1_indx,i)) * [1; -1] + axis_bias(min1_indx), 360);
    theta_candiates2 = mod(acosd(norm_rdoa(min2_indx,i)) * [1; -1] + axis_bias(min2_indx), 360);
    
    ang_diff = [theta_candiates1(1) - theta_candiates2; theta_candiates1(2) - theta_candiates2];
    [~, sorted_diff_indx] = sort(abs(ang_diff), 1);
    
    switch sorted_diff_indx(1)
        case 1
            theta_est(i) = mean([theta_candiates1(1), theta_candiates2(1)]);
        case 2
            theta_est(i) = mean([theta_candiates1(1), theta_candiates2(2)]);
        case 3
            theta_est(i) = mean([theta_candiates1(2), theta_candiates2(1)]);
        case 4
            theta_est(i) = mean([theta_candiates1(2), theta_candiates2(2)]);
    end
end



