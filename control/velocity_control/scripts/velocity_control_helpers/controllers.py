#!/usr/bin/python3

import numpy as np

np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning) 

def get_controller(controller_type: str):
  if controller_type == "pid":
    return PID
  elif controller_type == "linear_drag_model":
    assert 0, "Don't do this to yourself... Use PID"
    return LinearDragModel
  elif controller_type == "ipid":
    assert 0, "Don't do this to yourself... Use PID"
    return iPID
  else:
    raise ValueError("Invalid generator type")


class GenericController():
  def get_attitude_reference(self, v_ref: np.ndarray, v_actual: np.ndarray, timestamp: float):
    raise NotImplementedError

  def _clamp(self, value: float, limits: tuple) -> float:
    return np.min([np.max([value, limits[0]]), limits[1]])

  def get_delta_hat(self) -> tuple:
    return (0, 0)


class PID(GenericController):
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

    # Always want to use a single print, as other processes could interfere with
    # the print if more ROS-nodes are run in the same terminal
    info_str = 10*"=" + " PID selected. Control parameters: " + 10*"=" + "\n"
    info_str += f"Pitch: \tKp: {self.Kp_x} \tKi: {self.Ki_x} \tKd: {self.Kd_x} \tLimits: {self.pitch_limits}\n"
    info_str += f"Roll: \tKp: {self.Kp_y} \tKi: {self.Ki_y} \tKd: {self.Kd_y} \tLimits: {self.roll_limits}\n"
    info_str += 36*"="
    print(info_str)

    self.prev_ts = None
    self.error_int = np.zeros(2)
    self.prev_error = np.zeros(2)


  def get_attitude_reference(
      self, 
      v_ref : np.ndarray, 
      v     : np.ndarray, 
      ts    : float
    ) -> np.ndarray:
    error = (-v.ravel()[:2] + v_ref.ravel()[:2].T).reshape((2))

    error_surge = -error[0] # Negated to ensure correct angle definition
    error_sway = error[1]

    if self.prev_ts is not None and ts != self.prev_ts:
      dt = (ts - self.prev_ts).to_sec()

      error_surge_dot = (error_surge - self.prev_error[0]) / dt
      error_sway_dot = (error_sway - self.prev_error[1]) / dt

      self.prev_error = error

      # Avoid integral windup (not optimal)
      self.error_int[0] += error_surge * dt
      self.error_int[0] = self._clamp(self.error_int[0], self.pitch_limits)

      self.error_int[1] += error_sway * dt
      self.error_int[1] = self._clamp(self.error_int[1], self.roll_limits)

    else:
      error_surge_dot = error_sway_dot = 0

    self.prev_ts = ts

    pitch_reference = self.Kp_x * error_surge + self.Kd_x * error_surge_dot + self.Ki_x * self.error_int[0]
    roll_reference = self.Kp_y * error_sway + self.Kd_y * error_sway_dot + self.Ki_y * self.error_int[1]

    pitch_reference = self._clamp(pitch_reference, self.pitch_limits)
    roll_reference = self._clamp(roll_reference, self.roll_limits)

    attitude_reference = np.array([roll_reference, pitch_reference], dtype=float)

    return attitude_reference


class LinearDragModel(GenericController):
  def __init__(self, params: dict, limits: dict):
    super().__init__()

    self._m = params["drone_mass"]
    self._g = params["g"]
    self._d_x = params["dx"]
    self._d_y = params["dy"]

    pitch_limits = limits["pitch"]
    roll_limits = limits["roll"]

    self.pitch_limits = [lim * np.pi / 180.0 for lim in pitch_limits]
    self.roll_limits = [lim * np.pi / 180.0 for lim in roll_limits]

    print(10*"=", " Linear drag model selected. Control params:", 10*"=")
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
    pitch_ref = np.arctan(-(accel_x_desired / self._g + (self._d_x * vx) / (self._m * self._g)))
    roll_ref = np.arctan(accel_y_desired / self._g + (self._d_y * vy) / (self._m * self._g))

    pitch_ref = self._clamp(pitch_ref, self.pitch_limits)
    roll_ref = self._clamp(roll_ref, self.roll_limits)

    attitude_ref = np.array([roll_ref, pitch_ref])

    if debug:
      print(f"ts:{ts}\tRefs: R: {roll_ref:.3f}\tP: {pitch_ref:.3f}\tax_des: {accel_x_desired:.3f}\tay_des: {accel_y_desired:.3f}")
      print()

    return attitude_ref


class iPID(GenericController):
  def __init__(self, params: dict, limits: dict):
    super().__init__()

    self.Kp_x = params["x_axis"]["kp"]
    self.Ki_x = params["x_axis"]["ki"]
    self.Kd_x = params["x_axis"]["kd"]
    self.alpha_pitch = params["x_axis"]["alpha"]

    self.Kp_y = params["y_axis"]["kp"]
    self.Ki_y = params["y_axis"]["ki"]
    self.Kd_y = params["y_axis"]["kd"]
    self.alpha_roll = params["y_axis"]["alpha"]

    pitch_limits = limits["pitch"]
    roll_limits = limits["roll"]

    self.pitch_limits = [lim * np.pi / 180.0 for lim in pitch_limits]
    self.roll_limits = [lim * np.pi / 180.0 for lim in roll_limits]

    # Always want to use a single print, as other processes could interfere with
    # the print if more ROS-nodes are run in the same terminal
    info_str = 10*"=" + " iPID selected. Control parameters: " + 10*"=" + "\n"
    info_str += f"Pitch:\tKp: {self.Kp_x}\tKi: {self.Ki_x}\tKd: {self.Kd_x}\tAlpha: {self.alpha_pitch}\tLimits: {self.pitch_limits}\n"
    info_str += f"Roll:\tKp: {self.Kp_y}\tKi: {self.Ki_y}\tKd: {self.Kd_y}\tAlpha: {self.alpha_roll}\tLimits: {self.roll_limits}\n"
    info_str += 36*"="
    print(info_str)

    self.delta_hat_roll : float = 0
    self.delta_hat_pitch : float = 0

    self.prev_vel : np.ndarray = np.zeros(2)
    self.error_int : np.ndarray = np.zeros(2)
    self.prev_error : np.ndarray = np.zeros(2)
    self.prev_ts : float = None

    self.vel_dot_ref_lp : np.ndarray = np.zeros(2)
    self.vel_dot_lp : np.ndarray = np.zeros(2)


  def get_attitude_reference(
        self, 
        v_ref : np.ndarray, 
        v     : np.ndarray, 
        ts    : float
      ) -> np.ndarray:

    error = (-v[:2] + v_ref[:2].T).reshape((2))

    error_surge = error[0]
    error_sway = error[1]

    surge_dot_ref = v_ref[3][0]
    sway_dot_ref = v_ref[4][0]

    if self.prev_ts is not None and ts != self.prev_ts:
      dt = (ts - self.prev_ts).to_sec()

      error_surge_dot = (error_surge - self.prev_error[0]) / dt
      error_sway_dot = (error_sway - self.prev_error[1]) / dt

      # Avoid integral windup
      if self.pitch_limits[0] <= self.error_int[0] <= self.pitch_limits[1]:
        self.error_int[0] += error_surge * dt

      if self.roll_limits[0] <= self.error_int[0] <= self.roll_limits[1]:
        self.error_int[1] += error_sway * dt

      # Estimate accelerations
      surge_dot_hat = (v[0] - self.prev_vel[0]) / dt 
      sway_dot_hat = (v[1] - self.prev_vel[1]) / dt  
      
    else:
      error_surge_dot = error_sway_dot = 0 
      surge_dot_hat = sway_dot_hat = 0

    error_surge_dot_hat = surge_dot_ref - surge_dot_hat
    error_sway_dot_hat = sway_dot_ref - sway_dot_hat

    # Intelligent model-free control
    pd_pitch = self.Kp_x * error_surge + self.Kd_x * error_surge_dot
    pd_roll = self.Kp_y * error_sway + self.Kd_y

    pid_pitch = self.Kp_x * error_surge + self.Kd_x * error_surge_dot + self.Ki_x * self.error_int[0]
    pid_roll = self.Kp_y * error_sway + self.Kd_y * error_sway_dot + self.Ki_y * self.error_int[1]

    adaptive_pitch_ref = -surge_dot_ref + pid_pitch - self.delta_hat_pitch #(surge_dot_ref + pid_pitch - self.delta_hat_pitch)
    adaptive_roll_ref = -sway_dot_ref + pid_roll - self.delta_hat_roll #(sway_dot_ref + pid_roll - self.delta_hat_roll)

    adaptive_pitch_ref = self._clamp(adaptive_pitch_ref, self.pitch_limits)
    adaptive_roll_ref = self._clamp(adaptive_roll_ref, self.roll_limits)

    attitude_reference = np.array([adaptive_roll_ref, adaptive_pitch_ref], dtype=np.float)

    # Using MRAC on first order system, showed that delta_hat_dot = -gamma * error_axis, where gamma > 0 and error_axis = ref - measurement
    gamma = 0.01
    dt = 0.05
    delta_hat_pitch_dot = -self.delta_hat_pitch + gamma * error_surge
    delta_hat_roll_dot = -self.delta_hat_roll + gamma * error_sway 

    delta_hat_pitch_dot = gamma * error_surge
    delta_hat_roll_dot = gamma * error_sway 

    self.delta_hat_pitch = self.delta_hat_pitch + dt * delta_hat_pitch_dot 
    self.delta_hat_roll = self.delta_hat_roll + dt * delta_hat_roll_dot

    self.prev_error = np.array([error_surge, error_sway], dtype=np.float)
    self.prev_ts = ts

    return attitude_reference

  def get_delta_hat(self) -> tuple:
    return (self.delta_hat_roll, self.delta_hat_pitch)
