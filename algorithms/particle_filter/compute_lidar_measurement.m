function [measurement] = compute_lidar_measurement(map, pose, lidar_config)                                                                                                                                     
                                                                                                                                                                                                                  
  measurement = zeros(1, length(lidar_config));                                                                                                                                                                   
                                                                                                                                                                                                                  
  for i = 1:length(lidar_config)                                                                                                                                                                                  
      direction = pose(3) + lidar_config(i);
      intersections = ray_cast(pose(1:2), map.walls, direction);                                                                                                                                                  
                                                                                                                                                                                                                  
      if isempty(intersections)                                                                                                                                                                                   
          measurement(i) = NaN;                                                                                                                                                                                   
      else                                                                                                                                                                                                        
          % Vzdálenost od robota ke každému průsečíku
          dists = sqrt((intersections(:,1) - pose(1)).^2 + (intersections(:,2) - pose(2)).^2);                                                                                                                    
          measurement(i) = min(dists);
      end                                                                                                                                                                                                         
  end             
                                                                                                                                                                                                                  
  end         
