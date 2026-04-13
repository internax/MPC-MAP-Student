function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)                                                                                                                                     
                                                                                                                                                                                                                  
  T = read_only_vars.sampling_period;                                                                                                                                                                             
  d = read_only_vars.agent_drive.interwheel_dist;                                                                                                                                                                 
                                                                                                                                                                                                                  
  vR = motion_vector(1);                                                                                                                                                                                          
  vL = motion_vector(2);                                                                                                                                                                                          
                                                                                                                                                                                                                  
  % Kinematika diferenciálního pohonu                                                                                                                                                                             
  v     = (vR + vL) / 2;
  omega = (vR - vL) / d;                                                                                                                                                                                          
                                                                                                                                                                                                                  
  % Nová poloha
  new_x     = old_pose(1) + v * cos(old_pose(3)) * T;                                                                                                                                                             
  new_y     = old_pose(2) + v * sin(old_pose(3)) * T;                                                                                                                                                             
  new_theta = old_pose(3) + omega * T;
                                                                                                                                                                                                                  
  % Šum — bez něj by se částice pohybovaly identicky                                                                                                                                                              
  sigma_xy    = 0.02;
  sigma_theta = 0.01;                                                                                                                                                                                             
                  
  new_x     = new_x     + sigma_xy    * randn();                                                                                                                                                                  
  new_y     = new_y     + sigma_xy    * randn();
  new_theta = new_theta + sigma_theta * randn();                                                                                                                                                                  
                  
  new_pose = [new_x, new_y, new_theta];

  end  

