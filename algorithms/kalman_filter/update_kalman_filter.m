function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)

mu    = public_vars.mu;
sigma = public_vars.sigma;

% Predikce
u = public_vars.motion_vector;
[mu, sigma] = ekf_predict(mu, sigma, u, public_vars.kf, read_only_vars.sampling_period);

% Korekce - pokud je gnss k dispozici
z = read_only_vars.gnss_position;
if ~isnan(z(1))
    [mu, sigma] = kf_measure(mu, sigma, z', public_vars.kf);
end

end
