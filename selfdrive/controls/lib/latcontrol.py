import math
import csv
import numpy as np
import time
import json
from selfdrive.controls.lib.pid import PIController
from common.numpy_fast import interp
from common.realtime import sec_since_boot
from cereal import car
import os, os.path

_DT = 0.01    # 100Hz
_DT_MPC = 0.05  # 20Hz
_tuning_stage = 0


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
      self.reactance = CP.steerReactance
      self.resistance = CP.steerResistance
      self.inductance = CP.steerInductance
      self.ff_angle_factor = 1.0                                                     # Kf multiplier for angle-based feed forward
      self.ff_rate_factor = 10.0                                                      # Kf multiplier for rate-based feed forward
      # Eliminate break-points, since they aren't needed (and would cause problems for resonance)
      KpV = [np.interp(25.0, CP.steerKpBP, CP.steerKpV)]
      KiV = [np.interp(25.0, CP.steerKiBP, CP.steerKiV) * _DT / self.projection_factor]
      self.pid = PIController(([0.], KpV),
                              ([0.], KiV),
                              k_f=CP.steerKf, pos_limit=1.0)
    else:
      self.pid = PIController((CP.steerKpBP, CP.steerKpV),
                              (CP.steerKiBP, CP.steerKiV),
                              k_f=CP.steerKf, pos_limit=1.0)
      self.smooth_factor = 1.0
      self.projection_factor = 0.0
      self.accel_limit = 0.0
      self.ff_angle_factor = 1.0
      self.ff_rate_factor = 0.0
    self.last_cloudlog_t = 0.0
    self.prev_angle_rate = 0
    self.feed_forward = 0.0
    self.last_mpc_ts = 0.0
    self.angle_steers_des_time = 0.0
    self.angle_steers_des_mpc = 0.0
    self.steer_counter = 1.0
    self.steer_counter_prev = 0.0
    self.rough_steers_rate = 0.0
    self.prev_angle_steers = 0.0
    self.calculate_rate = True

    # variables for dashboarding
    DIR = '/sdcard/tuning'
    try:
      os.mkdir(DIR)
    except:
      pass

    self.filenumber = len([name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))])

    self.dash_file = open(DIR + '/dashboard_file_%d.csv' % self.filenumber, mode='w')
    self.dash_writer = csv.writer(self.dash_file, delimiter=',', quotechar='', quoting=csv.QUOTE_NONE)
    self.dash_writer.writerow(['angle_steers_des_mpc','angle_steers_des','mpc_time','cur_time','angle_steers','angle_rate','v_ego','steer_override',
                    'p','i','f','ff_type_a','ff_type_r','sway','reactance',
                    'inductance','resistance','eonToFront','time'])

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

    self.frames = 0
    self.curvature_factor = 0.0
    self.mpc_frame = 0

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


  def update(self, active, v_ego, angle_steers, angle_rate, angle_offset, steer_override, CP, VM, path_plan):

    if angle_rate == 0.0 and self.calculate_rate:
      if angle_steers != self.prev_angle_steers:
        self.steer_counter_prev = self.steer_counter
        self.rough_steers_rate = (self.rough_steers_rate + 100.0 * (angle_steers - self.prev_angle_steers) / self.steer_counter_prev) / 2.0
        self.steer_counter = 0.0
      elif self.steer_counter >= self.steer_counter_prev:
        self.rough_steers_rate = (self.steer_counter * self.rough_steers_rate) / (self.steer_counter + 1.0)
      self.steer_counter += 1.0
      angle_rate = self.rough_steers_rate

      # Don't use accelerated rate unless it's from CAN
      accelerated_angle_rate = angle_rate
    else:
      # If non-zero angle_rate is provided, use it instead
      self.calculate_rate = False
      # Use steering rate from the last 2 samples to estimate acceleration for a likely future steering rate
      accelerated_angle_rate = 2.0 * angle_rate - self.prev_angle_rate


    cur_time = sec_since_boot()
    self.angle_steers_des_time = cur_time

    if v_ego < 0.3 or not active:
      output_steer = 0.0
      self.feed_forward = 0.0
      self.pid.reset()
      self.angle_steers_des = angle_steers
    else:
      # Interpolate desired angle between MPC updates
      self.angle_steers_des = np.interp(cur_time, path_plan.mpcTimes, path_plan.mpcAngles)
      self.angle_steers_des_time = cur_time

      # Determine the target steer rate for desired angle, but prevent the acceleration limit from being exceeded
      # Restricting the steer rate creates the resistive component needed for resonance
      restricted_steer_rate = np.clip(self.angle_steers_des - float(angle_steers) , float(accelerated_angle_rate) - self.accel_limit,
                                                                                    float(accelerated_angle_rate) + self.accel_limit)

      # Determine projected desired angle that is within the acceleration limit (prevent the steering wheel from jerking)
      projected_angle_steers_des = self.angle_steers_des + self.projection_factor * restricted_steer_rate

      # Determine future angle steers using accellerated steer rate
      projected_angle_steers = float(angle_steers) + self.projection_factor * float(accelerated_angle_rate)

      steers_max = get_steer_max(CP, v_ego)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max
      deadzone = 0.0

      if CP.steerControlType == car.CarParams.SteerControlType.torque:
        # Decide which feed forward mode should be used (angle or rate).  Use more dominant mode, but only if conditions are met
        # Spread feed forward out over a period of time to make it inductive (for resonance)
        if abs(self.ff_rate_factor * float(restricted_steer_rate)) > abs(self.ff_angle_factor * float(self.angle_steers_des) - float(angle_offset)) - 0.5 \
            and (abs(float(restricted_steer_rate)) > abs(accelerated_angle_rate) or (float(restricted_steer_rate) < 0) != (accelerated_angle_rate < 0)) \
            and (float(restricted_steer_rate) < 0) == (float(self.angle_steers_des) - float(angle_offset) - 0.5 < 0):
          self.feed_forward = (((self.smooth_factor - 1.) * self.feed_forward) + self.ff_rate_factor * v_ego**2 * float(restricted_steer_rate)) / self.smooth_factor
          ff_type = "r"
        elif abs(self.angle_steers_des - float(angle_offset)) > 0.5:
          self.feed_forward = (((self.smooth_factor - 1.) * self.feed_forward) + self.ff_angle_factor * v_ego**2 \
                              * float(apply_deadzone(float(self.angle_steers_des) - float(angle_offset), 0.5))) / self.smooth_factor
          ff_type = "a"
        else:
          self.feed_forward = (((self.smooth_factor - 1.) * self.feed_forward) + 0.0) / self.smooth_factor
          ff_type = "a"

        # Use projected desired and actual angles instead of "current" values, in order to make PI more reactive (for resonance)
        output_steer = self.pid.update(projected_angle_steers_des, projected_angle_steers, check_saturation=(v_ego > 10),
                                        override=steer_override, feedforward=self.feed_forward, speed=v_ego, deadzone=deadzone)

      receiveTime = int(time.time() * 1000)
      self.dash_writer.writerow([str(round(path_plan.angleSteers, 2)),
                            str(round(self.angle_steers_des, 2)),
                            str(round(path_plan.mpcTimes[1], 4)),
                            str(round(cur_time, 4)),
                            str(round(angle_steers, 2)),
                            str(round(float(angle_rate), 1)),
                            str(round(v_ego, 1)),
                            1 if steer_override else 0,
                            str(round(self.pid.p, 4)),
                            str(round(self.pid.i, 4)),
                            str(round(self.pid.f, 4)),
                            1 if ff_type == "a" else 0, 1 if ff_type == "r" else 0,
                            #str(round(path_plan.sway,2)),
                            str(round(self.reactance,2)),
                            str(round(self.inductance,2)),
                            str(round(self.resistance,2)),
                            str(round(CP.eonToFront,1)),
                            str(receiveTime)])

    self.sat_flag = self.pid.saturated
    self.prev_angle_rate = angle_rate
    self.prev_angle_steers = angle_steers

    # return MPC angle in the unused output (for ALCA)
    if CP.steerControlType == car.CarParams.SteerControlType.torque:
      return output_steer, self.angle_steers_des
    else:
      return self.angle_steers_des_mpc, float(self.angle_steers_des)
