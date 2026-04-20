function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
    public_vars.pf_enabled    = 0;
    public_vars.motion_vector = [0, 0];
    public_vars.gnss_log      = [];

    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

    % Task 1 - manualne navrzena trajektorie: outdoor_1, start [2,2] -> cil [16,2]
    % Hustá cesta interpolovaná mezi waypointy (nutné pro lookahead controller)
    wpts = [2,2; 8,8; 14,8; 16,2];
    dense = [];
    for i = 1:size(wpts,1)-1
        n = 60;
        dense = [dense; linspace(wpts(i,1),wpts(i+1,1),n)', linspace(wpts(i,2),wpts(i+1,2),n)'];
    end
    public_vars.path = dense;
end

% Task 1 - inicializacni faze: sbir GNSS mereni pro odhad stredni hodnoty a kovariance
N_init = 100;
if ~public_vars.gnss_init_done
    z = read_only_vars.gnss_position;
    if ~isnan(z(1))
        public_vars.gnss_log = [public_vars.gnss_log; z];
    end

    if size(public_vars.gnss_log, 1) >= N_init
        gnss_mean = mean(public_vars.gnss_log);
        gnss_cov  = cov(public_vars.gnss_log);

        % Matice Q - kovariance sumu mereni GNSS
        % Nasobek > 1 snizuje vliv GNSS na odhad theta -> plynulejsi jizda
        public_vars.kf.Q = 50 * gnss_cov;

        % Task 4 - pocatecni belief z GNSS (neznama pocatecni poloha)
        % Orientace neni merena -> vysoka variance pi^2
        public_vars.mu    = [gnss_mean(1); gnss_mean(2); pi/2];
        public_vars.sigma = [gnss_cov, zeros(2,1); zeros(1,2), pi^2];
        % Task 3 - znama pocatecni poloha (vysoka jistota):
        % public_vars.mu    = [2; 2; pi/2];
        % public_vars.sigma = zeros(3, 3);

        public_vars.gnss_init_done = true;
    else
        % Robot stoji a ceka na dostatek GNSS dat
        public_vars.motion_vector = [0, 0];
        return;
    end
end

% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);

end
