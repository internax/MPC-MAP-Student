function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE EKF pose when GNSS available, particle filter mean otherwise

if public_vars.gnss_available
    % GNSS dostupne - pouzij EKF (presnejsi)
    estimated_pose = public_vars.mu';
else
    % GNSS nedostupne (indoor) - pouzij stredni hodnotu particle filteru
    estimated_pose = mean(public_vars.particles(:,1:3), 1);
end

end
