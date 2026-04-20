function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)
%KF_MEASURE KF correction step using GNSS measurement (linear observation)

C = kf.C;
Q = kf.Q;

% Inovace
innov = z - C * mu;

% Inov kovariance
S = C * sigma * C' + Q;

% Kalman gain
K = sigma * C' / S;

% State update
new_mu    = mu + K * innov;
new_mu(3) = atan2(sin(new_mu(3)), cos(new_mu(3))); %norm

% kovariance update
new_sigma = (eye(3) - K * C) * sigma;

end
