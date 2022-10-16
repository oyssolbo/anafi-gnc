#!/usr/bin/python3

import numpy as np 
import scipy 
import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import math 
from scipy.optimize import least_squares


class LSEstimateODEParameters:
  def __init__(self, y, u, p_hat_0, dt) -> None:
    self.y = y
    self.u = u 
    self.p0 = p_hat_0
    self.dt = dt
    self.x = 0

  def residuals(self, p_hat : np.ndarray) -> np.ndarray:
    """
    p_hat is the current parameter estimate
    p_hat = [tau_hat, k_hat]
    """
    tau_hat = p_hat[0]
    k_hat = p_hat[1] 

    errors = np.zeros(len(self.y))

    for i in range(len(self.u)):
      x_dot = self.f(tau_hat, k_hat, self.x, self.u[i])
      self.x = self.x + self.dt * x_dot 
      y_hat = self.x 

      errors[i] = y_hat - self.y[i]
      # print(y_hat)
      # print(self.y[i])
      # print()
      # if i > 90:
      #   import sys 
      #   import os 
      #   sys.exit(0)

    self.errors = errors
    
    return errors

  def optimize(self):
    optimization_results = least_squares(
      fun=self.residuals,
      x0=self.p0,
      method='lm'
    )
    # print(self.errors)
    # plt.plot(self.errors)
    # plt.show()
    return optimization_results.x


  def f(self, tau, k, x, u):
    return 1 / tau * (-x + k * u)


def f(tau, k, x, u):
  return 1 / tau * (-x + k * u)

def generate_test_ode(t_max : int, dt : float) -> np.ndarray:
  if dt < 1e-8:
    return None 
  if t_max < 0:
    t_max = -t_max

  num_steps = math.ceil(t_max / dt) 
  
  k_test = 1.5825
  tau_test = 1.39827

  u = np.empty((num_steps, 1))
  y = np.empty((num_steps, 1))

  x = 0
  
  for i in range(num_steps):
    u[i] = 10.0 - float(1 / (i + 1)) 
    x_dot = f(tau_test, k_test, x, u[i])
    x = x + dt * x_dot 

    y[i] = x

  return u, y


if __name__ == '__main__':
  dt = 0.1
  t_max = 10
  u_exact, y_exact = generate_test_ode(10, 0.1)

  u_noisy = u_exact #+ np.random.multivariate_normal(np.zeros_like(u_exact).ravel(), 0.25 * np.diag(np.ones_like(u_exact).ravel()))
  y_noisy = y_exact #+ np.random.multivariate_normal(np.zeros_like(y_exact).ravel(), 4 * np.diag(np.ones_like(u_exact).ravel()))

  # for idx in range(len(u_exact)):
  #   u_noisy[idx] += np.random.normal(0, 0.0625)
  #   y_noisy[idx] += np.random.normal(0, 0.25) 

  

  p_hat_0 = np.array([1.3, 1])
  ls_estimator = LSEstimateODEParameters(y_noisy, u_noisy, p_hat_0, dt)
  p_hat = ls_estimator.optimize()
  print(p_hat)

  tau_hat = p_hat[0]
  k_hat = p_hat[1] 

  y_hat_arr = np.zeros(len(y_noisy))
  x = 0

  for i in range(len(u_noisy)):
    print(u_noisy[i])
    x_dot = f(tau_hat, k_hat, x, u_noisy[i])
    x = x + dt * x_dot                    
    y_hat = x 
    print(x)

    y_hat_arr[i] = y_hat  

  y_diff = y_noisy.ravel() - y_hat_arr

  print(y_noisy.shape)
  print(y_hat_arr.shape)
  print(y_diff.shape)

  # plt.plot(y_exact)
  plt.plot(y_diff)
  plt.show()
