#!/usr/bin/env python3

import numpy as np

def Rzyx(rho_deg: float, theta_deg: float, psi_deg: float):
    return Rz(psi_deg) @ Ry(theta_deg) @ Rx(rho_deg)

def Rxyz(rho_deg: float, theta_deg: float, psi_deg: float):
    return Rx(rho_deg) @ Ry(theta_deg) @ Rz(psi_deg)

def Rx(degrees):
    radians = np.deg2rad(degrees)
    c = np.cos(radians)
    s = np.sin(radians)

    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]])

def Ry(degrees):
    radians = np.deg2rad(degrees)
    c = np.cos(radians)
    s = np.sin(radians)

    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])

def Rz(degrees):
    radians = np.deg2rad(degrees)
    c = np.cos(radians)
    s = np.sin(radians)

    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]])