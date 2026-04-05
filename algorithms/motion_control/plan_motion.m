function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

  pose = read_only_vars.mocap_pose;   % [x, y, uhl]
                                                                                                                                                                                                                     
  if isempty(pose) || isempty(public_vars.path)                                                                                                                                                                      
      public_vars.motion_vector = [0, 0];
      return                                                                                                                                                                                                         
  end             

  target = get_target(pose, public_vars.path);                                                                                                                                                                       
   
  % uhel                                                                                                                                                                                                     
  angle_to_target = atan2(target(2) - pose(2), target(1) - pose(1));                                                                                                                                                                                                               
  angle_error = angle_to_target - pose(3);                                                                                                                                                                           
  angle_error = atan2(sin(angle_error), cos(angle_error));                                                                                                                                                           
                                                                                                                                                                                                                     
  % param
  v_base = 0.5;                                                                                                                                                                              
  k = 0.5;                                                                                                                                                                                        
                                                                                                                                                                                                                  
  %rychlst kola                                                                                                                                                                                                  
  vR = v_base + k * angle_error;                                                                                                                                                                                     
  vL = v_base - k * angle_error;                                                                                                                                                                                     
   
  % limitace                                                                                                                                                                                          
  max_v = read_only_vars.agent_drive.max_vel;
  vR = max(-max_v, min(max_v, vR));                                                                                                                                                                                  
  vL = max(-max_v, min(max_v, vL));                                                                                                                                                                                  
                                                                                                                                                                                                                     
  public_vars.motion_vector = [vR, vL];                                                                                                                                                                              
           
  %disp("pose:"); disp(pose)                                                                                                                                                                                                         
      


end