function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Return KF mean as current pose estimate [x, y, theta]

if ~isempty(public_vars.mu)
    estimated_pose = public_vars.mu';
else
    estimated_pose = nan(1, 3);
end

end
