#!/usr/bin/python3

import numpy as np 
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import math 
from scipy.optimize import minimize
from bagpy import bagreader
from scipy.spatial.transform import Rotation
from functools import reduce

#rosbag = bagreader("/home/msccomputer/catkin_ws/src/anafi_msc/out/rosbag/lab/estimate_pitch_models/2022-10-18-14-54-19.bag")
rosbag = bagreader("/home/msccomputer/catkin_ws/src/anafi_msc/out/rosbag/lab/lab_testing_16_10_rosbag_attitude_roll_testing/2022-10-16-11-04-39.bag")

def drop_columns_with_name_in_df(df, name : str):
  return df.drop(columns=(df.filter(regex=name).columns)) 

def drop_columns_in_df(df):
  return df.drop(columns=["header.seq", "header.stamp.secs", "header.stamp.nsecs", "header.frame_id"])

def rename_columns(df, new_colum_names: list):
  column_names = ['Time'] + new_colum_names
  df.columns = column_names
  return df

def sync_dfs_based_on_time(dataframes: list):
    df_merged = reduce(lambda left,right: pd.merge_asof(left, right, on="Time", allow_exact_matches=False, direction="nearest"), dataframes)
    return df_merged

def initialize_time_at_0(df):
  t0 = df["Time"][0]
  df["Time"] = df["Time"] - t0 
  return df

def convert_quat_to_rpy(df):
  # Really inefficient
  new_dataframe = df[["Time"]].copy()
  rolls = []
  pitchs = []
  yaws = []

  for row, _ in df.iterrows():
    qx = df.loc[row, "quaternion.x"]
    qy = df.loc[row, "quaternion.y"]
    qz = df.loc[row, "quaternion.z"]
    qw = df.loc[row, "quaternion.w"]
    rpy = Rotation.from_quat([qx, qy, qz, qw]).as_euler("xyz", degrees=False)
    rolls.append(rpy[0])
    pitchs.append(rpy[1])
    yaws.append(rpy[2])
  new_dataframe["Roll"] = rolls
  new_dataframe["Pitch"] = pitchs
  new_dataframe["Yaw"] = yaws 
  return new_dataframe

def correct_offsets_rpy(df, offsets_rpy):
  df["roll"] = df["roll"] - offsets_rpy[0]
  df["pitch"] = df["pitch"] - offsets_rpy[1]
  df["yaw"] = df["yaw"] - offsets_rpy[2]
  return df

def extract_u(df):
  t_roll_cmd_start = 0
  t_pitch_cmd_start = 0# 183

  prev_roll_cmd = 0
  prev_pitch_cmd = 0

  t_roll_cmd_end = 147 #0 # hardcoded
  t_pitch_cmd_end = 0 # 242

  has_roll_flattened = False 
  has_pitch_flattened = False

  roll_commands = df["cmd_roll"]
  pitch_commands =df["cmd_pitch"]

  for index, row in df.iterrows():
    cmd_roll = row["cmd_roll"]
    cmd_pitch = row["cmd_pitch"]

    # print(np.abs(cmd_roll - prev_roll_cmd))

    if t_roll_cmd_start == 0:
      if np.abs(cmd_roll - prev_roll_cmd) > 1e-2:
        t_roll_cmd_start = index - 1
    elif not has_roll_flattened:
      if np.abs(cmd_roll - prev_roll_cmd) < 1e-4:
        has_roll_flattened = True 
    elif has_roll_flattened:
      # if np.abs(cmd_roll - prev_roll_cmd) > 1e-2:
      #   t_roll_cmd_end = index
      # hardcoded
      if cmd_roll < prev_roll_cmd - 1e-3:
        t_roll_cmd_end = index


    if t_pitch_cmd_start == 0:
      if np.abs(cmd_pitch - prev_pitch_cmd) > 1e-2:
        t_pitch_cmd_start = index - 1
    elif not has_pitch_flattened:
      if np.abs(cmd_pitch - prev_pitch_cmd) < 1e-4:
        has_pitch_flattened = True 
    elif has_pitch_flattened:
      if np.abs(cmd_pitch - prev_pitch_cmd) > 1e-2:
        t_pitch_cmd_end = index
      # hardcoded for positive values
      if cmd_pitch < prev_pitch_cmd - 1e-3:
        t_pitch_cmd_end = index

    if (t_roll_cmd_end != 0 or t_pitch_cmd_end != 0) and (t_roll_cmd_start != 0 or t_pitch_cmd_start != 0):
      break

    # print(index)

  if t_roll_cmd_end == 0 and t_pitch_cmd_end == 0:
    import warnings
    warnings.warn("Unable to detect end-point. Returning 0 for u")
    return np.zeros((1,1)), np.array([0,0]), ""
  if t_roll_cmd_end != 0:
    print("Roll")
    return roll_commands[t_roll_cmd_start:t_roll_cmd_end], np.array([t_roll_cmd_start, t_roll_cmd_end]), "roll"
  else:
    print("Pitch")
    return pitch_commands[t_pitch_cmd_start:t_pitch_cmd_end], np.array([t_pitch_cmd_start, t_pitch_cmd_end]), "pitch"


def extract_y(df, angle_str, start_idx, end_idx):
  return df[angle_str][start_idx:end_idx]

def get_dt(df):
  time = df["Time"]
  return time[1] - time[0]


def f(tau, k, x, u):
  return 1 / tau * (-x + k * u)


class LSEstimateODEParameters:
  def __init__(self, y, u, p_hat_0, dt) -> None:
    self.y = y
    self.u = u 
    self.p0 = p_hat_0
    self.dt = dt
    # self.x = 0

  def residuals(self, p_hat : np.ndarray) -> np.ndarray:
    """
    p_hat is the current parameter estimate
    p_hat = [tau_hat, k_hat]
    """
    tau_hat = p_hat[0]
    k_hat = p_hat[1] 

    errors = 0 
    x = 0
    for i in range(min(len(self.u), len(self.y))):
      x_dot = f(tau_hat, k_hat, x, self.u[i])
      x = x + self.dt * x_dot 
      y_hat = x 

      if np.isnan(y_hat - self.y[i]):
        break

      errors += np.abs(y_hat - self.y[i])  
    return errors

  def optimize(self):
    optimization_results = minimize(
      fun=self.residuals,
      x0=self.p0,
      method = 'Nelder-Mead'
    )
    print(optimization_results)
    return optimization_results.x


def generate_test_ode(t_max : int, dt : float, k_test : float, tau_test : float, x0 : float) -> np.ndarray:
  if dt < 1e-8:
    return None 
  if t_max < 0:
    t_max = -t_max

  num_steps = math.ceil(t_max / dt) 

  u = np.empty((num_steps, 1))
  y = np.empty((num_steps, 1))

  x = x0  
  for i in range(num_steps):
    u[i] = 10.0 #- float(1 / (i + 10)) * i 
    x_dot = f(tau_test, k_test, x, u[i])
    x = x + dt * x_dot 

    y[i] = x

  return u, y


if __name__ == '__main__':
  # dt = 0.1
  # t_max = 10
  x0 = 0

  # k_exact = 1.5273
  # tau_exact = 1.28274
  # u_exact, y_exact = generate_test_ode(t_max, dt, k_exact, tau_exact, x0)

  # u_noisy = u_exact.ravel() + np.random.multivariate_normal(np.zeros_like(u_exact).ravel(), 0.25 * np.diag(np.ones_like(u_exact).ravel()))
  # y_noisy = y_exact.ravel() + np.random.multivariate_normal(np.zeros_like(y_exact).ravel(), 2 * np.diag(np.ones_like(u_exact).ravel()))

  # for idx in range(len(u_exact)):
  #   u_noisy[idx] += np.random.normal(0, 0.0625)
  #   y_noisy[idx] += np.random.normal(0, 0.25) 

  offsets_rpy = (0.009875596168668191, 0.006219417359313843, 0)
  offset_y_timesteps = 6 # By visual observations

  attitude = rosbag.message_by_topic('/anafi/attitude')
  attitude = pd.read_csv(attitude) 

  attitude = drop_columns_in_df(attitude)
  attitude = convert_quat_to_rpy(attitude)
  attitude = rename_columns(attitude, ["roll", "pitch", "yaw"])   
  attitude = correct_offsets_rpy(attitude, offsets_rpy)

  cmd = rosbag.message_by_topic('/anafi/cmd_rpyt')
  cmd = pd.read_csv(cmd) 

  cmd = drop_columns_in_df(cmd)
  cmd = rename_columns(cmd, ["cmd_roll", "cmd_pitch", "cmd_yaw_rate", "cmd_thrust"])

  merged_frame = sync_dfs_based_on_time([attitude, cmd])
  merged_frame = initialize_time_at_0(merged_frame)
  # merged_frame.plot(x="Time", y=['cmd_roll', 'roll'])
  # merged_frame.plot(x="Time", y=['cmd_pitch', 'pitch'])
  # plt.show()

  dt = get_dt(merged_frame)
  # print(dt)

  df_u, start_end_indeces, angle_str = extract_u(merged_frame)
  df_y = extract_y(merged_frame, angle_str, start_end_indeces[0], start_end_indeces[1])
  angle_str = "roll"
  # print(df_y.to_numpy())
  # print(start_end_indeces)
  # df_y.plot()
  # df_u.plot()
  # plt.show()
  #df_u = merged_frame["cmd_pitch"]
  #df_y = merged_frame["pitch"]
  #df_y = df_y.shift(-offset_y_timesteps)
  u = df_u.to_numpy()
  y = df_y.to_numpy()
  y = y + np.ones_like(y) * offsets_rpy[1]

  k_hat =  0.85 
  tau_hat = 0.15

  p_hat_0 = np.array([tau_hat, k_hat])
  ls_estimator = LSEstimateODEParameters(y.ravel(), u, p_hat_0, dt)
  p_hat = ls_estimator.optimize()
  # tau_est = 0.3461490134251172
  # k_est = 0.8851490437093523
  tau_est = p_hat[0]
  k_est = p_hat[1]

  print("tau_est: {}".format(tau_est))
  print("k_est: {}".format(k_est))


  # print("tau: {}, tau_est: {}, tau_diff: {}".format(tau_exact, tau_est, tau_exact - tau_est))
  # print("k: {}, k_est: {}, k_diff: {}".format(k_exact, k_est, k_exact - k_est))

  # tau_hat = p_hat[0]
  # k_hat = p_hat[1] 

  y_hat_arr = np.zeros(len(y))
  x = 0

  for i in range(min(len(u), len(y))):
    x_dot = f(tau_est, k_est, x, u[i])
    x = x + dt * x_dot                    
    y_hat = x 

    y_hat_arr[i] = y_hat  

  plt.plot(y)
  plt.plot(y_hat_arr)
  plt.legend([f"measured {angle_str}", f"theoretical {angle_str}"])
  plt.title("Measured compared to theoretical {} solution. Using:\n tau = {}\n k = {}".format(angle_str, tau_est, k_est))
  plt.xlabel("timestep")
  plt.ylabel("angle [rad]")
  plt.show()
