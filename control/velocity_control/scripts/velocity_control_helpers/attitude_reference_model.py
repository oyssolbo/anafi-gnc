#!/usr/bin/python3

# This POS is written by Martin Falang (2021-2022)

import numpy as np

# def get_attitude_reference_generator(generator_type: str):
#   if generator_type == "pid":
#     return PIDReferenceGenerator
#   elif generator_type == "linear_drag_model":
#     return LinearDragModelReferenceGenerator
#   elif generator_type == "ipid":
#     return iPIDReferenceGenerator
#   else:
#     raise ValueError("Invalid generator type")

class GenericAttitudeReferenceGenerator():
  def get_attitude_reference(self, v_ref: np.ndarray, v_actual: np.ndarray, timestamp: float):
    raise NotImplementedError

  def clamp(self, value: float, limits: tuple) -> float:
    if value < limits[0]:
      return limits[0]
    elif value > limits[1]:
      return limits[1]
    else:
      return value

class PIDReferenceGenerator(GenericAttitudeReferenceGenerator):
  def __init__(self, params: dict, limits: dict):
    super().__init__()

    self.Kp_x = params["x_axis"]["kp"]
    self.Ki_x = params["x_axis"]["ki"]
    self.Kd_x = params["x_axis"]["kd"]

    self.Kp_y = params["y_axis"]["kp"]
    self.Ki_y = params["y_axis"]["ki"]
    self.Kd_y = params["y_axis"]["kd"]

    pitch_limits = limits["pitch"]
    roll_limits = limits["roll"]

    self.pitch_limits = [lim * np.pi / 180.0 for lim in pitch_limits]
    self.roll_limits = [lim * np.pi / 180.0 for lim in roll_limits]

    print(10*"=", "Control params", 10*"=")
    print(f"Pitch: \tKp: {self.Kp_x} \tKi: {self.Ki_x} \tKd: {self.Kd_x} \tLimits: {self.pitch_limits}")
    print(f"Roll: \tKp: {self.Kp_y} \tKi: {self.Ki_y} \tKd: {self.Kd_y} \tLimits: {self.roll_limits}")
    print(36*"=")

    self.prev_ts = None
    self.error_int = np.zeros(2)
    self.prev_error = np.zeros(2)

  def get_attitude_reference(
      self, 
      v_ref : np.ndarray, 
      v     : np.ndarray, 
      ts    : float, 
      debug : bool        = False
    ) -> np.ndarray:

    error = (-v[:2] + v_ref[:2])

    e_x = error[0]
    e_y = error[1]

    if self.prev_ts is not None and ts != self.prev_ts:
      dt = (ts - self.prev_ts).to_sec()

      e_dot_x = (e_x - self.prev_error[0]) / dt
      e_dot_y = (e_y - self.prev_error[1]) / dt

      self.prev_error = error

      # Avoid integral windup
      if self.pitch_limits[0] <= self.error_int[0] <= self.pitch_limits[1]:
        self.error_int[0] += e_x * dt

      if self.roll_limits[0] <= self.error_int[1] <= self.roll_limits[1]:
        self.error_int[1] += e_y * dt

    else:
      e_dot_x = e_dot_y = 0

    self.prev_ts = ts

    pitch_reference = self.Kp_x*e_x + self.Kd_x*e_dot_x + self.Ki_x*self.error_int[0]
    roll_reference = self.Kp_y*e_y + self.Kd_y*e_dot_y + self.Ki_y*self.error_int[1]

    pitch_reference = self.clamp(pitch_reference[0], self.pitch_limits)
    roll_reference = self.clamp(roll_reference[0], self.roll_limits)

    if np.abs(pitch_reference) < 1e-5:
      pitch_reference = 0
    if np.abs(roll_reference) < 1e-5:
      roll_reference = 0

    attitude_reference = np.array([roll_reference, pitch_reference], dtype=float)

    if debug:
      print(f"Timestamp: {ts}")
      # print(f"Pitch gains:\tP: {self.Kp_x*e_x:.3f}\tI: {self.Ki_x*self.error_int[0]:.3f}\tD: {self.Kd_x*e_dot_x:.3f} ")
      # print(f"Roll gains:\tP: {self.Kp_y*e_y:.3f}\tI: {self.Ki_y*self.error_int[1]:.3f}\tD: {self.Kd_y*e_dot_y:.3f} ")
      print(pitch_reference)
      print(roll_reference)
      print()

    return attitude_reference

class LinearDragModelReferenceGenerator(GenericAttitudeReferenceGenerator):
  def __init__(self, params: dict, limits: dict):
    super().__init__()

    self._m = params["drone_mass"]
    self._g = params["g"]
    self._d_x = params["dx"]
    self._d_y = params["dy"]

    self.pitch_limits = limits["pitch"]
    self.roll_limits = limits["roll"]

    print(10*"=", "Control params", 10*"=")
    print(f"Pitch:\tdx: {self._d_x}\tLimits: {self.pitch_limits}")
    print(f"Roll:\tdy: {self._d_y}\tLimits: {self.roll_limits}")
    print(36*"=")

  def get_attitude_reference(self, v_ref: np.ndarray, v: np.ndarray, ts: float, debug=False):
    vx = v[0]
    vy = v[1]

    vx_ref = v_ref[0]
    vy_ref = v_ref[1]

    accel_x_desired = vx_ref - vx
    accel_y_desired = vy_ref - vy

    # Negative on x axis due to inverse relationship between pitch angle and x-velocity
    pitch_ref = np.rad2deg(np.arctan(-(accel_x_desired / self._g + (self._d_x * vx) / (self._m * self._g))))
    roll_ref = np.rad2deg(np.arctan(accel_y_desired / self._g + (self._d_y * vy) / (self._m * self._g)))

    pitch_ref = self.clamp(pitch_ref, self.pitch_limits)
    roll_ref = self.clamp(roll_ref, self.roll_limits)

    attitude_ref = np.array([roll_ref, pitch_ref])

    if debug:
      print(f"ts:{ts}\tRefs: R: {roll_ref:.3f}\tP: {pitch_ref:.3f}\tax_des: {accel_x_desired:.3f}\tay_des: {accel_y_desired:.3f}")
      print()

    return attitude_ref

class iPIDReferenceGenerator(GenericAttitudeReferenceGenerator):
  def __init__(self, params: dict, limits: dict):
    super().__init__()

    self.Kp_x = params["x_axis"]["kp"]
    self.Ki_x = params["x_axis"]["ki"]
    self.Kd_x = params["x_axis"]["kd"]
    print(params["x_axis"])
    self.alpha_x = params["x_axis"]["alpha"]

    self.Kp_y = params["y_axis"]["kp"]
    self.Ki_y = params["y_axis"]["ki"]
    self.Kd_y = params["y_axis"]["kd"]
    self.alpha_y = params["y_axis"]["alpha"]

    self.pitch_limits = limits["pitch"]
    self.roll_limits = limits["roll"]

    print(10*"=", "Control params", 10*"=")
    print(f"Pitch:\tKp: {self.Kp_x}\tKi: {self.Ki_x}\tKd: {self.Kd_x}\tAlpha: {self.alpha_x}\tLimits: {self.pitch_limits}")
    print(f"Roll:\tKp: {self.Kp_y}\tKi: {self.Ki_y}\tKd: {self.Kd_y}\tAlpha: {self.alpha_y}\tLimits: {self.roll_limits}")
    print(36*"=")

    self.F_roll = 0
    self.F_pitch = 0

    self.error_int = np.zeros(2)
    self.prev_error = np.zeros(2)
    self.prev_ts: float = None

  def get_attitude_reference(
        self, 
        v_ref : np.ndarray, 
        v     : np.ndarray, 
        ts    : float, 
        debug : bool        = False
      ) -> np.ndarray:

    vx = v[0]
    vy = v[1]

    vx_ref = v_ref[0]
    vy_ref = v_ref[1]

    e_x = vx_ref - vx
    e_y = vy_ref - vy

    ax_star = v_ref[3]
    ay_star = v_ref[4]

    if self.prev_ts is not None and ts != self.prev_ts:
      dt = (ts - self.prev_ts).to_sec()

      e_dot_x = (e_x - self.prev_error[0]) / dt
      e_dot_y = (e_y - self.prev_error[1]) / dt

      # Avoid integral windup
      if self.pitch_limits[0] <= self.error_int[0] <= self.pitch_limits[1]:
        self.error_int[0] += e_x * dt

      if self.roll_limits[0] <= self.error_int[0] <= self.roll_limits[1]:
        self.error_int[1] += e_y * dt
    else:
      e_dot_x = e_dot_y = 0

    pitch_ref = (self.F_pitch - ax_star + self.Kp_x * e_x + self.Kd_x * e_dot_x + self.Ki_x * self.error_int[0]) / self.alpha_x
    roll_ref = (self.F_roll - ay_star + self.Kp_y * e_y + self.Kd_y * e_dot_y + self.Ki_y * self.error_int[1]) / self.alpha_y

    pitch_ref = self.clamp(pitch_ref, self.pitch_limits)
    roll_ref = self.clamp(roll_ref, self.roll_limits)

    attitude_reference = np.array([roll_ref, pitch_ref])

    self.F_pitch += (ax_star - self.alpha_x*pitch_ref - self.Kp_x*e_x - self.Kd_x * e_dot_x - self.Ki_x * self.error_int[0])
    self.F_roll += (ay_star - self.alpha_y*roll_ref - self.Kp_y*e_y - self.Kd_y * e_dot_y - self.Ki_y * self.error_int[1])

    self.prev_error = np.array([e_x, e_y])
    self.prev_ts = ts

    if debug:
      print(f"ts: {ts}")
      print(f"Refs:\tPitch: {pitch_ref:.3f}\tRoll: {roll_ref:.3f}")
      print(f"F pitch:\t{self.F_pitch}\tF roll:\t{self.F_roll}")
      print(f"ax_star: {ax_star}\tay_star: {ay_star}")
      print(f"ex: {e_x}\tey: {e_y}")
      print(f"Pitch gains:\tP: {self._Kp_pitch * e_x:.3f}\tI: {self._Ki_pitch * self.error_int[0]:.3f}\tD: {self._Kd_pitch * e_dot_x:.3f} ")
      print(f"Roll gains:\tP: {self._Kp_roll * e_y:.3f}\tI: {self._Ki_roll * self.error_int[1]:.3f}\tD: {self._Kd_roll * e_dot_y:.3f} ")
      print()

    return attitude_reference
