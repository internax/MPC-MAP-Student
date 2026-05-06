function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Proportional controller driving robot along path using EKF pose

  % Task 3/4: use EKF-estimated pose (not ground-truth mocap)
  pose = public_vars.estimated_pose;

  if isempty(pose) || any(isnan(pose)) || isempty(public_vars.path)
      public_vars.motion_vector = [0, 0];
      return
  end

  target = get_target(pose, public_vars.path);

  angle_to_target = atan2(target(2) - pose(2), target(1) - pose(1));
  angle_error     = angle_to_target - pose(3);
  angle_error     = atan2(sin(angle_error), cos(angle_error));

  k      = 0.3;
  v_base = 1.0 * (1 - 0.6 * abs(angle_error) / pi);

  vR = v_base + k * angle_error;
  vL = v_base - k * angle_error;

  max_v = read_only_vars.agent_drive.max_vel;
  vR = max(-max_v, min(max_v, vR));
  vL = max(-max_v, min(max_v, vL));

  public_vars.motion_vector = [vR, vL];

end
