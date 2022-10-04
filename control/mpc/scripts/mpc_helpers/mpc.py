from cmath import inf
import xdrlib
import casadi 
import do_mpc 
import numpy as np

from scipy.spatial.transform import Rotation

def ode(x : casadi.MX, u : casadi.MX):
  # Get these from a config-file
  drone_mass = 0.320
  gravitational_acceleration = 9.82179
  linear_drag_x = 0.08063504
  linear_drag_y = 0.09929089

  # Random values - must be estimated later with enough data (preferable from real world experiments)
  k_phi = 1
  k_theta = 1
  tau_phi = 0.01
  tau_theta = 0.01

  linear_drag_matrix = np.diag([linear_drag_x, linear_drag_y, 0.0])
  g_ned = np.array([[0], [0], [gravitational_acceleration]])

  # Extracting current states
  velocity_body = np.array(
    [
      [x[3]], 
      [x[4]], 
      [x[5]]
    ]
  )

  # This is an ugly an inefficient method
  # Doubles the complexity of the program 
  roll = x[6]
  pitch = x[7]
  yaw = x[8]

  cos_roll = casadi.cos(roll)
  sin_roll = casadi.sin(roll)

  cos_pitch = casadi.cos(pitch)
  sin_pitch = casadi.sin(pitch)

  cos_yaw = casadi.cos(yaw)
  sin_yaw = casadi.sin(yaw)

  rotation_matrix_x = np.array(
    [
      [1, 0, 0],
      [0, cos_roll, -sin_roll],
      [0, sin_roll, cos_roll]
    ]
  )
  rotation_matrix_y = np.array(
    [
      [cos_pitch, 0, sin_pitch],
      [0, 1, 0],
      [-sin_pitch, 0, cos_pitch]
    ]
  )
  rotation_matrix_z = np.array(
    [
      [cos_yaw, -sin_yaw, 0],
      [sin_yaw, cos_yaw, 0],
      [0, 0, 1]
    ]
  )
  rotation_matrix_body_to_vehicle = rotation_matrix_z @ rotation_matrix_y @ rotation_matrix_x
  rotation_matrix_body_to_vehicle = casadi.SX(rotation_matrix_body_to_vehicle)
  rotation_matrix_vehicle_to_body = casadi.transpose(rotation_matrix_body_to_vehicle)

  # rotation_matrix_body_to_vehicle = #Rotation.from_euler("xyz", np.array([x[6], x[7], x[8]])).as_matrix()
  #print(rotation_matrix_body_to_vehicle) # Rotation matrix may be NAN TODO

  # Derivatives
  d_pos = rotation_matrix_body_to_vehicle @ np.array(
    [
      [x[3]], 
      [x[4]], 
      [x[5]]
    ]
  )
  d_horizontal_vel = np.eye(2, 3) @ (((rotation_matrix_vehicle_to_body @ g_ned) - linear_drag_matrix @ velocity_body) / (drone_mass))
  d_vertical_vel = -u[3] # Change in velocity defined in NED, while the thrust is in ENU

  d_rpy = np.array(
    [
      [(k_phi * u[0] - x[6]) / tau_phi], 
      [(k_theta * u[1] - x[7]) / tau_theta], 
      [u[2]]
    ]
  )
  return casadi.vertcat(*[d_pos, d_horizontal_vel, d_vertical_vel, d_rpy])


def create_model() -> do_mpc.model.Model:
  model_type = "continuous"
  model = do_mpc.model.Model(model_type)

  x = model.set_variable(var_type='_x', var_name='x', shape=(9,1))
  u = model.set_variable(var_type='_u', var_name='u', shape=(4,1))

  model.set_rhs('x', ode(x, u))
  model.setup()

  return model


def create_objective_function(x, u):
  # Config file - TODO
  q = [2, 2, 0.5, 0.25, 0.25, 0.1, 0.5, 0.5, 0.01]
  r = [1, 1, 0.5, 1]
  
  xTQ = casadi.mtimes(casadi.transpose(x), casadi.SX(np.diag(q)))
  xTQx = casadi.mtimes(xTQ, x)

  # uTR = casadi.mtimes(casadi.transpose(u), casadi.SX(np.diag(r)))
  # uTRu = casadi.mtimes(uTR, u)

  lagrange_term = xTQx
  meyer_term = xTQx
  # r_term = uTRu
  r_term = r

  return lagrange_term, r_term, meyer_term


def create_mpc(model : do_mpc.model.Model) -> do_mpc.controller.MPC:
  mpc = do_mpc.controller.MPC(model)

  # https://www.do-mpc.com/en/latest/api/do_mpc.controller.MPC.set_param.html
  setup_mpc = {
      'n_horizon': 8,                   # Prediction horizon
      'n_robust': 0,                    # Robust horizon for a robust scenario MPC
      'open_loop': False,               # False will generate an individual cntrol input for each scenario. 
                                        # True will use the same control input for all scenarios 
      't_step': 0.2,                    # Time step of the MPC
      'use_terminal_bounds': False,     # Bool indicating if terminal constraints are used
      'collocation_deg': 4,             # Degree of collocation for continous models - https://www.do-mpc.com/en/latest/theory_orthogonal_collocation.html
      'nl_cons_single_slack': False,    # True will use slacking variables for soft constraints 
      'store_full_solution': False,
      'store_lagr_multiplier': False,
      'nlpsol_opts': {
        'ipopt.linear_solver': 'mumps', # 'MA27',  # Efficient linear solver
        'ipopt.print_level': 0,         # Suppress output
        'ipopt.sb': 'yes', 
        'print_time': 0
      }
  }
  mpc.set_param(**setup_mpc)
  lagrange_term, r_term, meyer_term = create_objective_function(
    x=model["x"], 
    u=model["u"]
  )

  # Configure objective function:
  mpc.set_objective(lterm=lagrange_term, mterm=meyer_term)
  mpc.set_rterm(u = np.array(r_term)) 

  # State and input bounds - use a config file for this
  mpc.bounds['lower', '_x', 'x'] = [-inf, -inf, -inf, -2, -2, -0.1, -10*np.pi/180.0, -10*np.pi/180.0, -inf]
  mpc.bounds['upper', '_x', 'x'] = [inf, inf, inf, 2, 2, 0.1, 10*np.pi/180.0, 10*np.pi/180.0, inf]
  
  mpc.bounds['lower', '_u', 'u'] = [-10*np.pi/180.0, -10*np.pi/180.0, -25*np.pi/180.0, -1.0]
  mpc.bounds['upper', '_u', 'u'] = [10*np.pi/180.0, 10*np.pi/180.0, 25*np.pi/180.0, 1.0]

  mpc.setup()

  return mpc



if __name__ == '__main__':  
  model = create_model()  
  mpc = create_mpc(model=model)

  x0 = np.ones((9, 1))
  u0 = np.zeros((4, 1))

  y = np.zeros((9, 1))
  dt = 0.02
  import datetime
  while True:
    
    x0 = x0 + dt * np.random.normal(np.zeros((9, 1)), dt * np.ones((9, 1)))
    first_time = datetime.datetime.now()
    u0 = mpc.make_step(x0)
    later_time = datetime.datetime.now()
    print(x0)
    print(u0)
    print(later_time - first_time)
    print()
