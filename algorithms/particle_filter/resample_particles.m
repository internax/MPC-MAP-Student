function [new_particles] = resample_particles(particles, weights)                                                                                                                                               
                                                                                                                                                                                                                  
  N = size(particles, 1);                                                                                                                                                                                         
  new_particles = zeros(N, 3);                                                                                                                                                                                    
                                                                                                                                                                                                                  
  % Systematic resampling                                                                                                                                                                                         
  cumulative = cumsum(weights);
  step = 1/N;                                                                                                                                                                                                     
  r = rand() * step;                                                                                                                                                                                              
   
  j = 1;                                                                                                                                                                                                          
  for i = 1:N     
      threshold = r + (i-1) * step;
      while cumulative(j) < threshold                                                                                                                                                                             
          j = j + 1;
      end                                                                                                                                                                                                         
      new_particles(i, :) = particles(j, :);
  end

  end

