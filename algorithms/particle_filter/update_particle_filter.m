function [particles] = update_particle_filter(read_only_vars, public_vars)
%UPDATE_PARTICLE_FILTER Summary of this function goes here

particles = public_vars.particles;

% Přeskočí první iteraci abych mohl zobrazit inicializační stav
if read_only_vars.counter == 1
    return
end

% Prediction
for i=1:size(particles, 1)
    particles(i,:) = predict_pose(particles(i,:), public_vars.motion_vector, read_only_vars);
end

% Correction
measurements = zeros(size(particles,1), length(read_only_vars.lidar_config));
for i=1:size(particles, 1)
    measurements(i,:) = compute_lidar_measurement(read_only_vars.map, particles(i,:), read_only_vars.lidar_config);
end
weights = weight_particles(measurements, read_only_vars.lidar_distances);

% Resampling
particles = resample_particles(particles, weights);

n_random = 50;                                                                                                                                                                                                  
limits = read_only_vars.map.limits;
particles(1:n_random, 1) = limits(1) + (limits(3)-limits(1)) * rand(n_random,1);                                                                                                                                
particles(1:n_random, 2) = limits(2) + (limits(4)-limits(2)) * rand(n_random,1);                                                                                                                                
particles(1:n_random, 3) = -pi + 2*pi * rand(n_random,1); 
end

