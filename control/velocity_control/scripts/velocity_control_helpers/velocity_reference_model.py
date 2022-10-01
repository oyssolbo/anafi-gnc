#!/usr/bin/python3

import numpy as np

class VelocityReferenceModel():
  """
  Second order reference model for velocity

  The states are:
    x = [u_ref, v_ref, u_ref_dot, v_ref_dot, w_ref]
    and the notation is following Fossen 2021
  """
  def __init__(
        self, 
        omegas: tuple, 
        zetas : tuple,
        T_z   : float,
        K_z   : float
      ) -> None:
    w_x = omegas[0]
    w_y = omegas[1]

    zeta_x = zetas[0]
    zeta_y = zetas[1]

    self.Ad = np.array([
        [0,         0,        1,              0,              0],
        [0,         0,        0,              1,              0],
        [-w_x**2,   0,       -2*zeta_x*w_x,   0,              0],
        [0,        -w_y**2,   0,             -2*zeta_y*w_y,   0],
        [0,         0,        0,              0,              -1 / T_z]
    ])

    self.Bd = np.array([
        [0,        0,       0],
        [0,        0,       0],
        [w_x**2,   0,       0],
        [0,        w_y**2,  0],
        [0,        0,       K_z / T_z]
    ])

  def get_filtered_reference(
        self, 
        xd_prev   : np.ndarray, 
        v_ref_raw : np.ndarray, 
        dt        : float
      ) -> np.ndarray:
    r = v_ref_raw.reshape((3, 1)) 

    xd_dot = self.Ad @ xd_prev + self.Bd @ r
    xd_next = xd_prev + dt * xd_dot

    return xd_next

