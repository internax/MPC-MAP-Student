function [weights] = weight_particles(particle_measurements, lidar_distances)

  N = size(particle_measurements, 1);
  log_weights = zeros(N, 1);

  sigma = 0.1; 

  for i = 1:N
      for j = 1:length(lidar_distances)
          z_real = lidar_distances(j);
          z_part = particle_measurements(i, j);
          
            %nan
          if isnan(z_real) || isnan(z_part)
              continue
          end

          % Log-váha = součet log pravděpodobností
          log_weights(i) = log_weights(i) + log(norm_pdf(z_real - z_part, 0, sigma) + 1e-300);
      end
  end

  % Převod z log-prostoru — odečteme maximum pro numerickou stabilitu
  log_weights = log_weights - max(log_weights);
  weights = exp(log_weights);

  % Normalizace
  weights = weights / sum(weights);

end
