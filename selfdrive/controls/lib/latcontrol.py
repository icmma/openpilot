from selfdrive.controls.lib.pid import PIController
from common.numpy_fast import interp
from cereal import car
from common.realtime import sec_since_boot


def get_steer_max(CP, v_ego):
  return interp(v_ego, CP.steerMaxBP, CP.steerMaxV)


def apply_deadzone(angle, deadzone):
  if angle > deadzone:
    angle -= deadzone
  elif angle < -deadzone:
    angle += deadzone
  else:
    angle = 0.
  return angle


class LatControl(object):
  def __init__(self, CP):
    self.ff_rate_factor = 3.0
    self.pid = PIController((CP.steerKpBP, CP.steerKpV),
                            (CP.steerKiBP, CP.steerKiV),
                            k_f=CP.steerKf, pos_limit=1.0)
    self.last_cloudlog_t = 0.0
    self.dampened_angle_steers = 0.
    self.angle_steers_des = 0.

  def reset(self):
    self.pid.reset()

  def update(self, active, v_ego, angle_steers, angle_rate, steer_override, CP, VM, path_plan, angle_offset=0.):
    if v_ego < 0.3 or not active:
      output_steer = 0.0
      self.pid.reset()
    else:
      self.angle_steers_des = interp(sec_since_boot(), path_plan.mpcTimes, path_plan.mpcAngles)
      projected_angle_steers = angle_steers + 0.5 * angle_rate
      self.dampened_angle_steers = (44. * self.dampened_angle_steers + projected_angle_steers) / 45.
      steers_max = get_steer_max(CP, v_ego)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max

      desired_rate = self.angle_steers_des - float(angle_steers)
      offset_desired_angle = apply_deadzone(self.angle_steers_des - angle_offset, 0.5)
      rate_more_significant = abs(self.ff_rate_factor * desired_rate) > abs(offset_desired_angle)
      rate_away_from_center = (abs(desired_rate) > abs(angle_rate) or (desired_rate < 0) != (angle_rate < 0))
      rate_same_direction_as_angle = (desired_rate < 0) == (offset_desired_angle < 0)
      rate_mode = rate_more_significant and rate_away_from_center and rate_same_direction_as_angle

      if rate_mode and False:
        feed_forward = self.ff_rate_factor * v_ego**2 * desired_rate
      else:
        feed_forward = offset_desired_angle * v_ego**2

      output_steer = self.pid.update(self.angle_steers_des, self.dampened_angle_steers, check_saturation=(v_ego > 10), override=steer_override,
                                    feedforward=feed_forward, speed=v_ego, deadzone=0.0)

    self.sat_flag = self.pid.saturated
    return output_steer, float(self.angle_steers_des)
