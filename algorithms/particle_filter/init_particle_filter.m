function [public_vars] = init_particle_filter(read_only_vars, public_vars)                                                                                                                                      
                  
  N = read_only_vars.max_particles;                                                                                                                                                                               
  limits = read_only_vars.map.limits; % [xmin, ymin, xmax, ymax] 

  disp(limits)
                                                                                                                                                                                                                  
  x     = limits(1) + (limits(3) - limits(1)) * rand(N, 1);                                                                                                                                                       
  y     = limits(2) + (limits(4) - limits(2)) * rand(N, 1);                                                                                                                                                       
  theta = -pi + 2*pi * rand(N, 1);                                                                                                                                                                                
                  
  public_vars.particles = [x, y, theta];                                                                                                                                                                          
                  
end
