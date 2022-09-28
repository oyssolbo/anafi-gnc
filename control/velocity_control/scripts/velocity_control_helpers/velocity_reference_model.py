#!/usr/bin/python3

import numpy as np

class VelocityReferenceModel():
  """
  Second order reference model for velocity
  """
  def __init__(
        self, 
        omegas: tuple, 
        zetas: tuple
      ) -> None:
    self._w_x = omegas[0]
    self._w_y = omegas[1]

    self._zeta_x = zetas[0]
    self._zeta_y = zetas[1]

    self._Ad = np.array([
        [0,               0,            1,                          0],
        [0,               0,            0,                          1],
        [-self._w_x**2,   0,           -2*self._zeta_x*self._w_x,   0],
        [0,              -self._w_y**2, 0,                         -2*self._zeta_y*self._w_y]
    ])

    self._Bd = np.array([
        [0,             0],
        [0,             0],
        [self._w_x**2,  0],
        [0,             self._w_y**2]
    ])

  def get_filtered_reference(
        self, 
        xd_prev   : np.ndarray, 
        v_ref_raw : np.ndarray, 
        dt        : float
      ) -> np.ndarray:
    r = v_ref_raw.reshape((2, 1)) # Fuck python for being fuckings useless

    xd_dot = self._Ad @ xd_prev + self._Bd @ r
    xd_next = xd_prev + dt * xd_dot

    return xd_next

