import numpy as np
from common.realtime import sec_since_boot
from selfdrive.controls.lib.pid import PIController
from common.numpy_fast import interp
from selfdrive.kegman_conf import kegman_conf
from cereal import car

_DT = 0.01    # 100Hz


def get_steer_max(CP, v_ego):
  return interp(v_ego, CP.steerMaxBP, CP.steerMaxV)

def apply_deadzone(angle, deadzone):
  if angle > deadzone:
    angle -= deadzone
  elif angle < - deadzone:
    angle += deadzone
  else:
    angle = 0.
  return angle


class LatControl(object):
  def __init__(self, CP):

    kegman = kegman_conf()
    self.write_conf = False
    self.gernbySteer = True
    kegman.conf['tuneGernby'] = str(1)
    self.write_conf = True
    if kegman.conf['tuneGernby'] == "-1":
      kegman.conf['tuneGernby'] = str(1)
      self.write_conf = True
    if kegman.conf['reactSteer'] == "-1":
      kegman.conf['reactSteer'] = str(round(CP.steerReactTime,3))
      self.write_conf = True
    if kegman.conf['dampSteer'] == "-1":
      kegman.conf['dampSteer'] = str(round(CP.steerDampTime,3))
      self.write_conf = True
    if kegman.conf['reactMPC'] == "-1":
      kegman.conf['reactMPC'] = str(round(CP.steerMPCReactTime,3))
      self.write_conf = True
    if kegman.conf['dampMPC'] == "-1":
      kegman.conf['dampMPC'] = str(round(CP.steerMPCDampTime,3))
      self.write_conf = True
    if kegman.conf['Kp'] == "-1":
      kegman.conf['Kp'] = str(round(CP.steerKpV[0],3))
      self.write_conf = True
    if kegman.conf['Ki'] == "-1":
      kegman.conf['Ki'] = str(round(CP.steerKiV[0],3))
      self.write_conf = True

    if self.write_conf:
      kegman.write_config(kegman.conf)

    self.mpc_frame = 0
    self.total_desired_projection = max(0.0, CP.steerMPCReactTime + CP.steerMPCDampTime)
    self.total_actual_projection = max(0.0, CP.steerReactTime + CP.steerDampTime)
    self.actual_smoothing = max(1.0, CP.steerDampTime / _DT)
    self.desired_smoothing = max(1.0, CP.steerMPCDampTime / _DT)
    self.ff_angle_factor = 5.0
    self.ff_rate_factor = 0.1
    self.dampened_desired_angle = 0.0
    self.steer_counter = 1.0
    self.steer_counter_prev = 0.0
    self.rough_steers_rate = 0.0
    self.prev_angle_steers = 0.0
    self.calculate_rate = True
    self.feed_forward = 0.0
    self.rate_mode = 0.0
    self.angle_mode = 0.0

    KpV = [interp(25.0, CP.steerKpBP, CP.steerKpV)]
    KiV = [interp(25.0, CP.steerKiBP, CP.steerKiV)]
    self.pid = PIController(([0.], KpV),
                            ([0.], KiV),
                            k_f=CP.steerKf, pos_limit=1.0)

  def live_tune(self, CP):
    self.mpc_frame += 1
    if self.mpc_frame % 300 == 0:
      # live tuning through /data/openpilot/tune.py overrides interface.py settings
      kegman = kegman_conf()
      if kegman.conf['tuneGernby'] == "1":
        self.steerKpV = np.array([float(kegman.conf['Kp'])])
        self.steerKiV = np.array([float(kegman.conf['Ki'])])
        self.total_actual_projection = max(0.0, float(kegman.conf['dampSteer']) + float(kegman.conf['reactSteer']))
        self.total_desired_projection = max(0.0, float(kegman.conf['dampMPC']) + float(kegman.conf['reactMPC']))
        self.actual_smoothing = max(1.0, float(kegman.conf['dampSteer']) / _DT)
        self.desired_smoothing = max(1.0, float(kegman.conf['dampMPC']) / _DT)
        self.gernbySteer = (self.total_actual_projection > 0 or self.actual_smoothing > 1 or self.total_desired_projection > 0 or self.desired_smoothing > 1)

        # Eliminate break-points, since they aren't needed (and would cause problems for resonance)
        KpV = [interp(25.0, CP.steerKpBP, self.steerKpV)]
        KiV = [interp(25.0, CP.steerKiBP, self.steerKiV)]
        self.pid._k_i = ([0.], KiV)
        self.pid._k_p = ([0.], KpV)
        print(self.total_desired_projection, self.desired_smoothing, self.total_actual_projection, self.actual_smoothing, self.gernbySteer)
      else:
        self.gernbySteer = False
      self.mpc_frame = 0


  def reset(self):
    self.pid.reset()

  def update(self, active, v_ego, angle_steers, angle_rate, steer_override, CP, VM, path_plan):

    self.live_tune(CP)
    if angle_rate == 0.0 and self.calculate_rate:
      if angle_steers != self.prev_angle_steers:
        self.steer_counter_prev = self.steer_counter
        self.rough_steers_rate = (self.rough_steers_rate + 100.0 * (angle_steers - self.prev_angle_steers) / self.steer_counter_prev) / 2.0
        self.steer_counter = 0.0
      elif self.steer_counter >= self.steer_counter_prev:
        self.rough_steers_rate = (self.steer_counter * self.rough_steers_rate) / (self.steer_counter + 1.0)
      self.steer_counter += 1.0
      angle_rate = self.rough_steers_rate
      self.prev_angle_steers = float(angle_steers)
    else:
      # If non-zero angle_rate is provided, stop calculating rate
      self.calculate_rate = False

    if v_ego < 0.3 or not active:
      output_steer = 0.0
      self.pid.reset()
      self.dampened_angle_steers = float(angle_steers)
      self.dampened_desired_angle = float(angle_steers)
      self.dampened_desired_rate = 0.0
      self.dampened_rate_ff = 0.0
      self.dampened_angle_ff = 0.0
    else:

      if self.gernbySteer == False:
        self.dampened_angle_steers = float(angle_steers)
        self.dampened_desired_angle = float(path_plan.angleSteers)
        self.dampened_desired_rate = float(path_plan.rateSteers)

      else:
        cur_time = sec_since_boot()
        projected_desired_angle = interp(cur_time + self.total_desired_projection, path_plan.mpcTimes, path_plan.mpcAngles)
        self.dampened_desired_angle += ((projected_desired_angle - self.dampened_desired_angle) / self.desired_smoothing)
        projected_desired_rate = interp(cur_time + self.total_desired_projection, path_plan.mpcTimes, path_plan.mpcRates)
        self.dampened_desired_rate += ((projected_desired_rate - self.dampened_desired_rate) / self.desired_smoothing)

        if not steer_override:
          projected_angle_steers = float(angle_steers) + self.total_actual_projection * float(angle_rate)
        else:
          projected_angle_steers = float(angle_steers)
        self.dampened_angle_steers += ((projected_angle_steers - self.dampened_angle_steers) / self.actual_smoothing)

      if CP.steerControlType == car.CarParams.SteerControlType.torque:

        steers_max = get_steer_max(CP, v_ego)
        self.pid.pos_limit = steers_max
        self.pid.neg_limit = -steers_max
        deadzone = 0.0

        ff_target_angle = self.dampened_desired_angle - path_plan.angleOffset
        self.angle_mode = interp(abs(ff_target_angle), [1.5, 6.0], [0.0, 1.0])
        self.rate_mode = 1.0 - self.angle_mode
        angle_feedforward = self.angle_mode * self.ff_angle_factor * ff_target_angle
        rate_feedforward = self.rate_mode * self.ff_rate_factor * self.dampened_desired_rate
        feed_forward = v_ego**2 * (rate_feedforward + angle_feedforward)

        output_steer = self.pid.update(self.dampened_desired_angle, self.dampened_angle_steers, check_saturation=(v_ego > 10),
                                      override=steer_override, feedforward=feed_forward, speed=v_ego, deadzone=deadzone)

    self.sat_flag = self.pid.saturated
    self.prev_angle_steers = float(angle_steers)

    if CP.steerControlType == car.CarParams.SteerControlType.torque:
      return float(output_steer), float(path_plan.angleSteers)
    else:
      return float(self.dampened_desired_angle), float(path_plan.angleSteers)
