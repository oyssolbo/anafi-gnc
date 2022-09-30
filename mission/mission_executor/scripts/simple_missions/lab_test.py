#!/usr/bin/python3

import numpy as np

class Actions:
  def __init__(self) -> None:
    self.action_str = ""
    self.action_frame = "ned"
    self.action_movement : np.ndarray = np.zeros((4, 1))
  
  def get_action_str(self) -> str:
    return self.action_str
  
  def get_action_movement(self) -> np.ndarray:
    return self.action_movement

  def get_action_frame(self) -> str:
    return self.action_frame

class TakeoffAction(Actions):
  def __init__(self, altitude : float) -> None:
    super().__init__()

    self.action_str = "takeoff"
    self.action_movement[2] = altitude

class LandAction(Actions):
  def __init__(self) -> None:
    super().__init__()

    self.action_str = "land"

class TrackAction(Actions):
  def __init__(self) -> None:
    super().__init__()

    self.action_str = "track"

class MoveRelativeAction(Actions):
  def __init__(self, dpose : np.ndarray) -> None:
    super().__init__()
    if dpose.shape[0] != 4:
      dpose = dpose.reshape((4, 1))

    self.action_str = "move_relative"
    self.action_movement = dpose

class TravelToAction(Actions):
  # Fitted to travelling with respect to another frame
  pass 


def generate_actions(mission_id : int) -> list:#(Actions):
  action_list = []

  if mission_id == -2:
    action_list.append(TakeoffAction(altitude=2))
    action_list.append(MoveRelativeAction(np.array([0.5, 0, 0, 0]).T))
    action_list.append(MoveRelativeAction(np.array([0, 0.5, 0, 0]).T))
    action_list.append(TrackAction())
    action_list.append(LandAction())

  elif mission_id == -1:
    print("Only takeoff")
    action_list.append(TakeoffAction(altitude=1))

  elif mission_id == 0:
    print("Simplest action for takeoff and landing")
    action_list.append(TakeoffAction(altitude=2))
    action_list.append(TrackAction()) # No land

  elif mission_id == 1:
    print("Simplest action including tracking of platform")
    action_list.append(TakeoffAction(altitude=2))
    action_list.append(TrackAction())
    action_list.append(LandAction())

  elif mission_id == 2:
    print("Moving relative in a square")
    action_list.append(TakeoffAction(altitude=2))
    action_list.append(MoveRelativeAction(np.array([0.5, 0, 0, 0]).T))
    action_list.append(MoveRelativeAction(np.array([0, 0.5, 0, 0]).T))
    action_list.append(MoveRelativeAction(np.array([-1, 0, 0, 0]).T))
    action_list.append(MoveRelativeAction(np.array([0, -1, 0, 0]).T))
    action_list.append(MoveRelativeAction(np.array([0.5, 0, 0, 0]).T))
    action_list.append(MoveRelativeAction(np.array([0, 0.5, 0, 0]).T))
    action_list.append(TrackAction())
    action_list.append(LandAction())

  elif mission_id == 3:
    print("Rotating and varying altitude before tracking")
    action_list.append(TakeoffAction(altitude=1))
    action_list.append(MoveRelativeAction(np.array([0, 0, 1, 0]).T))
    action_list.append(MoveRelativeAction(np.array([0, 0.5, -0.5, 0]).T))
    action_list.append(MoveRelativeAction(np.array([0, -0.5, 0, 0]).T))
    action_list.append(MoveRelativeAction(np.array([0, 0, 0, np.pi/2.0]).T))
    action_list.append(TrackAction())
    action_list.append(LandAction())

  elif mission_id == 4:
    print("Varying altitude before tracking")
    action_list.append(TakeoffAction(altitude=1))
    action_list.append(MoveRelativeAction(np.array([0, 0, 1, 0]).T))
    action_list.append(MoveRelativeAction(np.array([0, 0.5, -0.5, 0]).T))
    action_list.append(MoveRelativeAction(np.array([0, -0.5, 2, 0]).T))

    action_list.append(MoveRelativeAction(np.array([0, 0, 1, 0]).T))
    action_list.append(MoveRelativeAction(np.array([0, 0, -0.5, 0]).T))
    action_list.append(MoveRelativeAction(np.array([0, 0, 3, 0]).T))
    action_list.append(MoveRelativeAction(np.array([0, 0, 20, 0]).T))
    action_list.append(MoveRelativeAction(np.array([0, 0, -10, 0]).T))

    action_list.append(TrackAction())
    action_list.append(LandAction())

  else:
    print("Moving relative with large magnitudes")
    action_list.append(TakeoffAction(altitude=1))
    action_list.append(MoveRelativeAction(np.array([0, 0, -1, 0]).T))
    action_list.append(MoveRelativeAction(np.array([10, 5, 0, 0]).T))
    action_list.append(MoveRelativeAction(np.array([20, 10, 0, 0]).T))

    action_list.append(MoveRelativeAction(np.array([30, 15, -1, 0]).T))
    action_list.append(MoveRelativeAction(np.array([-10, -15, 0, 0]).T))
    action_list.append(MoveRelativeAction(np.array([-20, -10, 0, 0]).T))
    action_list.append(MoveRelativeAction(np.array([-30, -5, 0, 0]).T))
    action_list.append(MoveRelativeAction(np.array([0, 0, -1, 0]).T))

    action_list.append(TrackAction())
    action_list.append(LandAction())


  return action_list
