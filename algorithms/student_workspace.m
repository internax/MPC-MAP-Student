function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
    public_vars.pf_enabled       = 0;
    public_vars.motion_vector    = [0, 0];
    public_vars.gnss_log         = [];
    public_vars.gnss_available   = false;
    public_vars.gnss_prev        = false; % pro detekci prechodu

    public_vars = init_kalman_filter(read_only_vars, public_vars);

    % particles inicializujeme pozdeji - az pri vjezdu do indoor zony
    public_vars.particles = zeros(read_only_vars.max_particles, 3);

    public_vars.path = [];
end

% Zjisti aktualni dostupnost GNSS
z = read_only_vars.gnss_position;
public_vars.gnss_available = ~isnan(z(1));

% Task 1 - inicializacni faze: sbir GNSS mereni pro odhad stredni hodnoty a kovariance
N_init = 100;
if ~public_vars.gnss_init_done
    if public_vars.gnss_available
        public_vars.gnss_log = [public_vars.gnss_log; z];
    end

    if size(public_vars.gnss_log, 1) >= N_init
        gnss_mean = mean(public_vars.gnss_log);
        gnss_cov  = cov(public_vars.gnss_log);

        public_vars.kf.Q = gnss_cov;

        % Task 4 - pocatecni belief z GNSS (neznama pocatecni poloha)
        public_vars.mu    = [gnss_mean(1); gnss_mean(2); pi/2];
        public_vars.sigma = [gnss_cov, zeros(2,1); zeros(1,2), pi^2];

        public_vars.gnss_init_done = true;
    else
        % Robot stoji a ceka na dostatek GNSS dat
        public_vars.motion_vector = [0, 0];
        public_vars.gnss_prev = public_vars.gnss_available;
        return;
    end
end

% Detekce prechodu GNSS dostupne -> nedostupne (vjezd do indoor zony)
entering_indoor = public_vars.gnss_prev && ~public_vars.gnss_available;
if entering_indoor
    % Inicializuj particles okolo aktualniho EKF odhadu
    N = read_only_vars.max_particles;
    std_xy    = 0.5;
    std_theta = 0.3;
    mu = public_vars.mu;
    public_vars.particles = [
        mu(1) + std_xy    * randn(N, 1), ...
        mu(2) + std_xy    * randn(N, 1), ...
        mu(3) + std_theta * randn(N, 1)
    ];
    public_vars.pf_enabled = 1;
end

% Detekce navratu do GNSS zone
if ~public_vars.gnss_prev && public_vars.gnss_available
    public_vars.pf_enabled = 0;
end

public_vars.gnss_prev = public_vars.gnss_available;

% 9. Update particle filter - pouze v indoor zone (bez GNSS)
if public_vars.pf_enabled
    public_vars.particles = update_particle_filter(read_only_vars, public_vars);
end

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);

end
