function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

target = get_target(public_vars.estimated_pose, public_vars.path);


% II. Compute motion vector
 if read_only_vars.counter <= 1
        public_vars.motion_vector = [0.1, 0.1];
        public_vars.position_turnpoint = 0;
        public_vars.raf = 0;
 end

    
switch public_vars.raf
    case 0
        if read_only_vars.counter == (public_vars.position_turnpoint + 600)
            public_vars.motion_vector = [0, 0.1];
            public_vars.position_turnpoint = read_only_vars.counter;
            public_vars.raf = 1;
        end

    case 1
         if read_only_vars.counter == (public_vars.position_turnpoint + 30)
            public_vars.motion_vector = [0.1, 0.1];
            public_vars.position_turnpoint = read_only_vars.counter;
            public_vars.raf = 2;
         end

    case 2
         if read_only_vars.counter == (public_vars.position_turnpoint + 400)
            public_vars.motion_vector = [0, 0.1];
            public_vars.position_turnpoint = read_only_vars.counter;
            public_vars.raf = 3;
         end

    case 3
         if read_only_vars.counter == (public_vars.position_turnpoint + 30)
            public_vars.motion_vector = [0.1, 0.1];
            public_vars.position_turnpoint = read_only_vars.counter;
            public_vars.raf = 4;
         end

    case 4
         if read_only_vars.counter == (public_vars.position_turnpoint + 600)
            public_vars.motion_vector = [0.1, 0];
            public_vars.position_turnpoint = read_only_vars.counter;
            public_vars.raf = 5;
         end

    case 5
         if read_only_vars.counter == (public_vars.position_turnpoint + 30)
            public_vars.motion_vector = [0.1, 0.1];
            public_vars.position_turnpoint = read_only_vars.counter;
            public_vars.raf = 6;
         end
   
    case 6
         if read_only_vars.counter == (public_vars.position_turnpoint + 250)
            public_vars.motion_vector = [0.1, 0];
            public_vars.position_turnpoint = read_only_vars.counter;
            public_vars.raf = 7;
         end

    case 7
         if read_only_vars.counter == (public_vars.position_turnpoint + 25)
            public_vars.motion_vector = [0.1, 0.1];
            public_vars.position_turnpoint = read_only_vars.counter;
            public_vars.raf = 8;
        end
     
end
    


end