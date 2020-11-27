function [est_pos] = Moore_Penrose(centers, doa)
% Estimate position given sensor position and direction of arrival angle

dim = size(centers,1);
nmeas = size(centers,2);

u = [cosd(doa); sind(doa)]; % unit vectors
term1_sum = zeros(dim,dim);
term2_sum = zeros(dim,1);

for i = 1:nmeas
    term1 = eye(dim) - (u(:,i) * u(:,i).');
    term2 = term1 * centers(:,i);
    term1_sum = term1_sum + term1;
    term2_sum = term2_sum + term2;
end

est_pos = inv(term1_sum) * term2_sum;

end

