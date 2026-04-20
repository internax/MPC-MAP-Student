function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, dt)

L  = kf.L;
vR = u(1);
vL = u(2);

v     = (vR + vL) / 2;
omega = (vR - vL) / L;
theta = mu(3); %akt. odhad 

new_mu    = mu + [v*cos(theta)*dt; v*sin(theta)*dt; omega*dt];
new_mu(3) = atan2(sin(new_mu(3)), cos(new_mu(3))); %normalizace 

% Jakobian
G = [1, 0, -v*sin(theta)*dt;
     0, 1,  v*cos(theta)*dt;
     0, 0,  1              ];

% EKF 
new_sigma = G * sigma * G' + kf.R;

end
