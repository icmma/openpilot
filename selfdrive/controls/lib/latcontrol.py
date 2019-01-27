import zmq
import math
import numpy as np
import time
import json
from selfdrive.controls.lib.pid import PIController
from selfdrive.controls.lib.drive_helpers import MPC_COST_LAT
from selfdrive.controls.lib.lateral_mpc import libmpc_py
from common.numpy_fast import interp
from common.realtime import sec_since_boot
from selfdrive.swaglog import cloudlog
from cereal import car

_DT = 0.01    # 100Hz
_DT_MPC = 0.05  # 20Hz
_tuning_stage = 0

def calc_states_after_delay(states, v_ego, steer_angle, curvature_factor, steer_ratio, delay, long_camera_offset):
  states[0].x = max(0.0, v_ego * delay + long_camera_offset)
  states[0].psi = v_ego * curvature_factor * math.radians(steer_angle) / steer_ratio * delay
  return states

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

    if CP.steerResistance > 0 and CP.steerReactance >= 0 and CP.steerInductance > 0:
      self.smooth_factor = CP.steerInductance * 2.0 * CP.steerActuatorDelay / _DT    # Multiplier for inductive component (feed forward)
      self.projection_factor = CP.steerReactance * CP.steerActuatorDelay / 2.0       # Mutiplier for reactive component (PI)
      self.accel_limit = 2.0 / CP.steerResistance                                    # Desired acceleration limit to prevent "whip steer" (resistive component)
      self.ff_angle_factor = 1.0                                                     # Kf multiplier for angle-based feed forward
      self.ff_rate_factor = 10.0                                                      # Kf multiplier for rate-based feed forward
      # Eliminate break-points, since they aren't needed (and would cause problems for resonance)
      KpV = np.interp(25.0, CP.steerKpBP, CP.steerKpV)
      self.KiV = np.interp(25.0, CP.steerKiBP, CP.steerKiV)
      self.pid = PIController(([0.], [KpV]),
                              ([0.], [self.KiV * _DT / self.projection_factor]),
                              k_f=CP.steerKf, pos_limit=1.0)
    else:
      self.pid = PIController((CP.steerKpBP, CP.steerKpV),
                              (CP.steerKiBP, CP.steerKiV),
                              k_f=CP.steerKf, pos_limit=1.0)
      self.smooth_factor = 1.0                   # Disabled
      self.projection_factor = 0.0               # Disabled
      self.accel_limit = 0.0                     # Disabled
      self.ff_angle_factor = 1.0                 # Disabled
      self.ff_rate_factor = 0.0                  # Disabled

    self.prev_angle_rate = 0.0
    self.inductance = CP.steerInductance
    self.reactance = CP.steerReactance
    self.resistance = CP.steerResistance
    self.resistanceIndex = 0
    self.inductanceIndex = 0
    self.reactanceIndex = 0
    self.feed_forward = 0.0
    self.steerActuatorDelay = CP.steerActuatorDelay
    self.last_cloudlog_t = 0.0
    self.setup_mpc(CP.steerRateCost)
    self.last_mpc_ts = 0.0
    self.angle_steers_des = 0.0
    self.angle_steers_des_mpc = 0.0
    self.angle_steers_des_time = 0.0
    self.avg_angle_steers = 0.0
    self.projected_angle_steers = 0.0
    self.left_change = 0.0
    self.right_change = 0.0
    self.path_change = 0.0
    self.prob_adjust = 0.0
    self.steer_counter = 1.0
    self.steer_counter_prev = 0.0
    self.angle_steers = 0.0
    self.angle_steers_rate = 0.0
    self.rough_steers_rate = 0.0
    self.rough_steers_rate_increment = 0.0
    self.prev_angle_steers = 0.0
    self.calculate_rate = True
    self.lane_change_rate = 0.0
    self.prev_lane_offset = 0.0
    self.prev_lane_offset_time = 0.0
    self.lane_offset_adjustment = 0.0

    # variables for dashboarding
    self.context = zmq.Context()
    self.steerpub = self.context.socket(zmq.PUB)
    self.steerpub.bind("tcp://*:8594")
    self.influxString = 'steerData3,testName=none,active=%s,ff_type=%s ff_type_a=%s,ff_type_r=%s,sway=%s,reactance=%s,inductance=%s,resistance=%s,eonToFront=%s,' \
           'prev_lane_offset=%s,lane_offset_adjustment=%s,lane_change_rate=%s,mpc_age=%s,angle_rate=%s,angle_steers=%s,angle_steers_des=%s,angle_steers_des_mpc=%s,v_ego=%s,c_poly[3]=%s,l_poly[3]=%s,r_poly[3]=%s,p=%s,i=%s,f=%s %s\n~'


    self.sine_wave = [ 0.0175, 0.0349, 0.0523, 0.0698, 0.0872, 0.1045, 0.1219, 0.1392, 0.1564, 0.1736, 0.1908, 0.2079, 0.225, 0.2419, 0.2588, 0.2756,
                      0.2924, 0.309, 0.3256, 0.342, 0.3584, 0.3746, 0.3907, 0.4067, 0.4226, 0.4384, 0.454, 0.4695, 0.4848, 0.5, 0.515, 0.5299, 0.5446,
                      0.5592, 0.5736, 0.5878, 0.6018, 0.6157, 0.6293, 0.6428, 0.6561, 0.6691, 0.682, 0.6947, 0.7071, 0.7193, 0.7314, 0.7431, 0.7547, 0.766,
                      0.7771, 0.788, 0.7986, 0.809, 0.8192, 0.829, 0.8387, 0.848, 0.8572, 0.866, 0.8746, 0.8829, 0.891, 0.8988, 0.9063, 0.9135, 0.9205,
                      0.9272, 0.9336, 0.9397, 0.9455, 0.9511, 0.9563, 0.9613, 0.9659, 0.9703, 0.9744, 0.9781, 0.9816, 0.9848, 0.9877, 0.9903, 0.9925,
                      0.9945, 0.9962, 0.9976, 0.9986, 0.9994, 0.9998, 1, 0.9998, 0.9994, 0.9986, 0.9976, 0.9962, 0.9945, 0.9925, 0.9903, 0.9877, 0.9848,
                      0.9816, 0.9781, 0.9744, 0.9703, 0.9659, 0.9613, 0.9563, 0.9511, 0.9455, 0.9397, 0.9336, 0.9272, 0.9205, 0.9135, 0.9063, 0.8988,
                      0.891, 0.8829, 0.8746, 0.866, 0.8572, 0.848, 0.8387, 0.829, 0.8192, 0.809, 0.7986, 0.788, 0.7771, 0.766, 0.7547, 0.7431, 0.7314,
                      0.7193, 0.7071, 0.6947, 0.682, 0.6691, 0.6561, 0.6428, 0.6293, 0.6157, 0.6018, 0.5878, 0.5736, 0.5592, 0.5446, 0.5299, 0.515,
                      0.5, 0.4848, 0.4695, 0.454, 0.4384, 0.4226, 0.4067, 0.3907, 0.3746, 0.3584, 0.342, 0.3256, 0.309, 0.2924, 0.2756, 0.2588, 0.2419,
                      0.225, 0.2079, 0.1908, 0.1736, 0.1564, 0.1392, 0.1219, 0.1045, 0.0872, 0.0698, 0.0523, 0.0349, 0.0175, 0.0, -0.0175, -0.0349, -0.0523,
                      -0.0698, -0.0872, -0.1045, -0.1219, -0.1392, -0.1564, -0.1736, -0.1908, -0.2079, -0.225, -0.2419, -0.2588, -0.2756, -0.2924, -0.309,
                      -0.3256, -0.342, -0.3584, -0.3746, -0.3907, -0.4067, -0.4226, -0.4384, -0.454, -0.4695, -0.4848, -0.5, -0.515, -0.5299, -0.5446,
                      -0.5592, -0.5736, -0.5878, -0.6018, -0.6157, -0.6293, -0.6428, -0.6561, -0.6691, -0.682, -0.6947, -0.7071, -0.7193, -0.7314,
                      -0.7431, -0.7547, -0.766, -0.7771, -0.788, -0.7986, -0.809, -0.8192, -0.829, -0.8387, -0.848, -0.8572, -0.866, -0.8746, -0.8829,
                      -0.891, -0.8988, -0.9063, -0.9135, -0.9205, -0.9272, -0.9336, -0.9397, -0.9455, -0.9511, -0.9563, -0.9613, -0.9659, -0.9703,
                      -0.9744, -0.9781, -0.9816, -0.9848, -0.9877, -0.9903, -0.9925, -0.9945, -0.9962, -0.9976, -0.9986, -0.9994, -0.9998, -1, -0.9998,
                      -0.9994, -0.9986, -0.9976, -0.9962, -0.9945, -0.9925, -0.9903, -0.9877, -0.9848, -0.9816, -0.9781, -0.9744, -0.9703, -0.9659,
                      -0.9613, -0.9563, -0.9511, -0.9455, -0.9397, -0.9336, -0.9272, -0.9205, -0.9135, -0.9063, -0.8988, -0.891, -0.8829, -0.8746,
                      -0.866, -0.8572, -0.848, -0.8387, -0.829, -0.8192, -0.809, -0.7986, -0.788, -0.7771, -0.766, -0.7547, -0.7431, -0.7314,
                      -0.7193, -0.7071, -0.6947, -0.682, -0.6691, -0.6561, -0.6428, -0.6293, -0.6157, -0.6018, -0.5878, -0.5736, -0.5592, -0.5446,
                      -0.5299, -0.515, -0.5, -0.4848, -0.4695, -0.454, -0.4384, -0.4226, -0.4067, -0.3907, -0.3746, -0.3584, -0.342, -0.3256,
                      -0.309, -0.2924, -0.2756, -0.2588, -0.2419, -0.225, -0.2079, -0.1908, -0.1736, -0.1564, -0.1392, -0.1219, -0.1045, -0.0872,
                      -0.0698, -0.0523, -0.0349, -0.0175, 0.0]

    self.steerdata = self.influxString
    self.frames = 0
    self.curvature_factor = 0.0
    self.mpc_frame = 0

  def setup_mpc(self, steer_rate_cost):
    self.libmpc = libmpc_py.libmpc
    self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, steer_rate_cost)

    self.mpc_solution = libmpc_py.ffi.new("log_t *")
    self.cur_state = libmpc_py.ffi.new("state_t *")
    self.mpc_angles = [0.0, 0.0, 0.0]
    self.mpc_times = [0.0, 0.0, 0.0]
    self.mpc_updated = False
    self.mpc_nans = False
    self.cur_state[0].x = 0.0
    self.cur_state[0].y = 0.0
    self.cur_state[0].psi = 0.0
    self.cur_state[0].delta = 0.0

  def reset(self):
    self.pid.reset()

  def roll_tune(self, CP, PL):
    self.mpc_frame += 1
    sway_index = self.mpc_frame % 2000
    if sway_index < 90:
      PL.PP.sway = (self.sine_wave[sway_index * 2]) * 0.35
    elif 90 <= sway_index < 180:
      PL.PP.sway = -(self.sine_wave[(sway_index - 180) * 4]) * 0.45

    if _tuning_stage == 1:
      if self.mpc_frame % 40 == 0:
        self.resistanceIndex += 1
        self.resistance = CP.steerResistance * (1.0 + 0.5 * self.sine_wave[self.resistanceIndex % 360])
        self.accel_limit = 2.0 / self.resistance
      if self.mpc_frame % 50 == 0:
        self.reactanceIndex += 1
        self.reactance = CP.steerReactance * (1.0 + 0.5 * self.sine_wave[self.reactanceIndex % 360])
        self.projection_factor = self.reactance * CP.steerActuatorDelay / 2.0
        self.pid._k_i = ([0.], [self.KiV * _DT / self.projection_factor])
      if self.mpc_frame % 60 == 0:
        self.inductanceIndex += 1
        self.inductance = CP.steerInductance * (1.0 + 0.5 * self.sine_wave[self.inductanceIndex % 360])
        self.smooth_factor = self.inductance * 2.0 * CP.steerActuatorDelay / _DT
    elif _tuning_stage == 2:
      if self.mpc_frame % 100 == 0:
        self.reactanceIndex += 1
        self.reactance = CP.steerReactance * (1.0 + 0.3 * self.sine_wave[self.reactanceIndex % 360])
        self.projection_factor = self.reactance * CP.steerActuatorDelay / 2.0
        self.pid._k_i = ([0.], [self.KiV * _DT / self.projection_factor])
      if self.mpc_frame % 125 == 0:
        self.inductanceIndex += 1
        self.inductance = CP.steerInductance * (1.0 + 0.3 * self.sine_wave[self.inductanceIndex % 360])
        self.smooth_factor = self.inductance * 2.0 * CP.steerActuatorDelay / _DT
    elif _tuning_stage == 3:
      if self.mpc_frame % 150 == 0:
        self.reactanceIndex += 1
        self.reactance = CP.steerReactance * (1.0 + 0.3 * self.sine_wave[self.reactanceIndex % 360])
        self.projection_factor = self.reactance * CP.steerActuatorDelay / 2.0
        self.pid._k_i = ([0.], [self.KiV * _DT / self.projection_factor])
    elif _tuning_stage == 4:
      if self.mpc_frame % 150 == 0:
        self.inductanceIndex += 1
        self.inductance = CP.steerInductance * (1.0 + 0.3 * self.sine_wave[self.inductanceIndex % 360])
        self.smooth_factor = self.inductance * 2.0 * CP.steerActuatorDelay / _DT
    elif _tuning_stage == 5:
      if self.mpc_frame % 150 == 0:
        self.resistanceIndex += 1
        self.resistance = CP.steerResistance * (1.0 + 0.3 * self.sine_wave[self.resistanceIndex % 360])
        self.accel_limit = 2.0 / self.resistance

  def update(self, active, v_ego, angle_steers, angle_rate, steer_override, d_poly, angle_offset, CP, VM, PL):
    self.mpc_updated = False

    if angle_rate == 0.0 and self.calculate_rate:
      if angle_steers != self.prev_angle_steers:
        self.steer_counter_prev = self.steer_counter
        self.rough_steers_rate = (self.rough_steers_rate + 100.0 * (angle_steers - self.prev_angle_steers) / self.steer_counter_prev) / 2.0
        self.steer_counter = 0.0
      self.steer_counter += 1.0

      if self.steer_counter > self.steer_counter_prev:
        self.rough_steers_rate = (self.steer_counter * self.rough_steers_rate) / (self.steer_counter + 1.0)

      angle_rate = self.rough_steers_rate
      accelerated_angle_rate = angle_rate

    else:
      # Use steering rate from the last 2 samples to estimate acceleration for a more realistic future steering rate
      accelerated_angle_rate = 2.0 * angle_rate - self.prev_angle_rate
      self.calculate_rate = False

    # TODO: this creates issues in replay when rewinding time: mpc won't run
    if self.last_mpc_ts < PL.last_md_ts:
      self.last_mpc_ts = PL.last_md_ts
      cur_time = sec_since_boot()
      mpc_time = float(self.last_mpc_ts / 1000000000.0)
      self.curvature_factor = VM.curvature_factor(v_ego)

      # Determine future angle steers using accelerated steer rate
      self.projected_angle_steers = float(angle_steers) + CP.steerActuatorDelay * float(accelerated_angle_rate)

      # Determine a proper delay time that includes the model's processing time, which is variable
      plan_age = _DT_MPC + cur_time - mpc_time
      total_delay = CP.steerActuatorDelay + plan_age

      self.l_poly = libmpc_py.ffi.new("double[4]", list(PL.PP.l_poly))
      self.r_poly = libmpc_py.ffi.new("double[4]", list(PL.PP.r_poly))
      self.c_poly = libmpc_py.ffi.new("double[4]", list(PL.PP.c_poly))
      self.p_poly = libmpc_py.ffi.new("double[4]", list(PL.PP.p_poly))

      # account for actuation delay and the age of the plan
      self.cur_state = calc_states_after_delay(self.cur_state, v_ego, self.projected_angle_steers, self.curvature_factor, CP.steerRatio, total_delay, CP.eonToFront)

      v_ego_mpc = max(v_ego, 5.0)  # avoid mpc roughness due to low speed

      self.libmpc.run_mpc(self.cur_state, self.mpc_solution,
                          self.l_poly, self.r_poly, self.p_poly,
                          PL.PP.l_prob, PL.PP.r_prob, PL.PP.p_prob, self.curvature_factor, v_ego_mpc, PL.PP.lane_width)

      self.mpc_updated = True

      #  Check for infeasable MPC solution
      self.mpc_nans = np.any(np.isnan(list(self.mpc_solution[0].delta)))
      if not self.mpc_nans:
        self.mpc_angles = [self.angle_steers_des,
                          float(math.degrees(self.mpc_solution[0].delta[1] * CP.steerRatio) + angle_offset),
                          float(math.degrees(self.mpc_solution[0].delta[2] * CP.steerRatio) + angle_offset)]

        self.mpc_times = [self.angle_steers_des_time,
                          mpc_time + _DT_MPC,
                          mpc_time + _DT_MPC + _DT_MPC]

        self.angle_steers_des_mpc = self.mpc_angles[1]

      else:
        self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, CP.steerRateCost)
        self.cur_state[0].delta = math.radians(angle_steers) / CP.steerRatio

        if cur_time > self.last_cloudlog_t + 5.0:
          self.last_cloudlog_t = cur_time
          cloudlog.warning("Lateral mpc - nan: True")

    elif self.frames > 0:
      self.steerpub.send(self.steerdata)
      self.frames = 0
      self.steerdata = self.influxString

    if v_ego < 0.3 or not active:
      output_steer = 0.0
      self.ateer_vibrate = 0.0
      self.feed_forward = 0.0
      self.pid.reset()
      self.angle_steers_des = angle_steers
      self.avg_angle_steers = angle_steers
      self.cur_state[0].delta = math.radians(angle_steers - angle_offset) / CP.steerRatio
    else:
      cur_time = sec_since_boot()

      if v_ego > 20 and _tuning_stage > 0: self.roll_tune(CP, PL)

      # Interpolate desired angle between MPC updates
      self.angle_steers_des = np.interp(cur_time, self.mpc_times, self.mpc_angles)
      self.angle_steers_des_time = cur_time
      self.avg_angle_steers = (4.0 * self.avg_angle_steers + angle_steers) / 5.0
      self.cur_state[0].delta = math.radians(self.angle_steers_des - angle_offset) / CP.steerRatio

      # Determine the target steer rate for desired angle, but prevent the acceleration limit from being exceeded
      # Restricting the steer rate creates the resistive component needed for resonance
      restricted_steer_rate = np.clip(self.angle_steers_des - float(angle_steers) , float(accelerated_angle_rate) - self.accel_limit, float(accelerated_angle_rate) + self.accel_limit)

      # Determine projected desired angle that is within the acceleration limit (prevent the steering wheel from jerking)
      projected_angle_steers_des = self.angle_steers_des + self.projection_factor * restricted_steer_rate

      # Determine future angle steers using steer rate
      self.projected_angle_steers = float(angle_steers) + self.projection_factor * float(accelerated_angle_rate)

      steers_max = get_steer_max(CP, v_ego)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max

      if CP.steerControlType == car.CarParams.SteerControlType.torque:
        # Decide which feed forward mode should be used (angle or rate).  Use more dominant mode, and only if conditions are met
        # Spread feed forward out over a period of time to make it more inductive (for resonance)
        if abs(self.ff_rate_factor * float(restricted_steer_rate)) > abs(self.ff_angle_factor * float(self.angle_steers_des) - float(angle_offset)) - 0.5 \
            and (abs(float(restricted_steer_rate)) > abs(accelerated_angle_rate) or (float(restricted_steer_rate) < 0) != (accelerated_angle_rate < 0)) \
            and (float(restricted_steer_rate) < 0) == (float(self.angle_steers_des) - float(angle_offset) - 0.5 < 0):
          ff_type = "r"
          self.feed_forward = (((self.smooth_factor - 1.) * self.feed_forward) + self.ff_rate_factor * v_ego**2 * float(restricted_steer_rate)) / self.smooth_factor
        elif abs(self.angle_steers_des - float(angle_offset)) > 0.5:
          ff_type = "a"
          self.feed_forward = (((self.smooth_factor - 1.) * self.feed_forward) + self.ff_angle_factor * v_ego**2 * float(apply_deadzone(float(self.angle_steers_des) - float(angle_offset), 0.5))) / self.smooth_factor
        else:
          ff_type = "r"
          self.feed_forward = (((self.smooth_factor - 1.) * self.feed_forward) + 0.0) / self.smooth_factor
      else:
        self.feed_forward = self.angle_steers_des   # feedforward desired angle
      deadzone = 0.0

      # Use projected desired and actual angles instead of "current" values, in order to make PI more reactive (for resonance)
      output_steer = self.pid.update(projected_angle_steers_des, self.projected_angle_steers, check_saturation=(v_ego > 10), override=steer_override,
                                     feedforward=self.feed_forward, speed=v_ego, deadzone=deadzone, freeze_integrator=steer_override | (_tuning_stage in (1,2,4)))

      # Hide angle error if being overriden
      if steer_override:
        self.projected_angle_steers = self.mpc_angles[1]
        self.avg_angle_steers = self.mpc_angles[1]

      # All but the last 3 lines after here are for real-time dashboarding
      steering_control_active = 0.0
      driver_torque = 0.0
      steer_status = 0.0
      steer_stock_torque = 0.0
      steer_stock_torque_request = 0.0
      self.angle_rate_desired = 0.0
      self.observed_ratio = 0.0
      capture_all = True
      if self.mpc_updated or capture_all:
        self.frames += 1
        self.steerdata += ("%d,%s,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d|" % \
          (1, ff_type, 1 if ff_type == "a" else 0, 1 if ff_type == "r" else 0, PL.PP.sway, self.reactance,self.inductance,self.resistance,CP.eonToFront, \
          self.prev_lane_offset, self.lane_offset_adjustment, self.lane_change_rate, cur_time - float(self.last_mpc_ts / 1000000000.0), float(angle_rate), angle_steers, self.angle_steers_des, self.mpc_angles[1], v_ego, \
          self.c_poly[3], self.l_poly[3], self.r_poly[3], self.pid.p, self.pid.i, self.pid.f, int(time.time() * 100) * 10000000))

    self.sat_flag = self.pid.saturated
    self.prev_angle_rate = angle_rate
    self.prev_angle_steers = angle_steers

    if CP.steerControlType == car.CarParams.SteerControlType.torque:
      return output_steer, float(self.angle_steers_des_mpc)
    else:
      return float(self.angle_steers_des_mpc), float(self.angle_steers_des)
