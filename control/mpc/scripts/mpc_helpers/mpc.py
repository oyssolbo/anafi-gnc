from cmath import inf
import xdrlib
import casadi 
import do_mpc 
import numpy as np

from scipy.spatial.transform import Rotation

class MPCSolver():
  def __init__(self, mpc_parameters : dict) -> None:

    # Extracting system parameters
    system_parameters = mpc_parameters["system_parameters"]

    drone_parameters = system_parameters["drone"]
    self.drone_mass = drone_parameters["mass"]

    aerodynamic_drag_parameters = drone_parameters["aerodynamic_linear_drag_coefficients"] 
    linear_drag_coefficient_x = aerodynamic_drag_parameters["x"]
    linear_drag_coefficient_y = aerodynamic_drag_parameters["y"]
    linear_drag_coefficient_z = aerodynamic_drag_parameters["z"]

    attitude_control_parameters = drone_parameters["attitude_control_parameters"]
    time_constants = attitude_control_parameters["time_constants"]
    input_gains = attitude_control_parameters["input_gains"]
    self.tau_roll = time_constants["roll"]
    self.tau_pitch = time_constants["pitch"]
    self.k_roll = input_gains["roll"]
    self.k_pitch = input_gains["pitch"]

    gravitational_constant = system_parameters["gravitational_acceleration"]

    self.linear_drag_matrix = np.diag([linear_drag_coefficient_x, linear_drag_coefficient_y, linear_drag_coefficient_z])
    self.g_ned = np.array([[0], [0], [gravitational_constant]])
    
    assert self.drone_mass > 0, "Mass must be positive"
    assert self.tau_pitch > 0 and self.tau_pitch > 0, "Time constants must be positive"
    assert self.k_pitch > 0 and self.k_pitch > 0, "Input gains must be positive" 
    assert linear_drag_coefficient_x > 0 and linear_drag_coefficient_y > 0, "Linear drag coefficients must be positive" 


    # Extracting tuning parameters
    tuning_parameters = mpc_parameters["tuning_parameters"]
    self.nx = tuning_parameters["nx"]
    self.nu = tuning_parameters["nu"]

    self.q_diagonal = tuning_parameters["objective_function"]["q_diagonal"]
    self.r_diagonal = tuning_parameters["objective_function"]["r_diagonal"]

    self.prediction_horizon = tuning_parameters["prediction_horizon"]
    self.robust_horizon = tuning_parameters["robust_horizon"]
    self.time_step = tuning_parameters["time_step"]

    assert len(self.q_diagonal) == self.nx, "Number of states must match the size of the Q-matrix"
    assert len(self.r_diagonal) == self.nu, "Number of inputs must match the size of the R-matrix"
    assert isinstance(self.prediction_horizon, int) and self.prediction_horizon >= 1, "Prediction horizon must an integer at least be 1"
    assert isinstance(self.robust_horizon, int) and self.robust_horizon >= 0, "Robust horizon must be an integer at least be 0"
    assert isinstance(self.time_step, float) and self.time_step > 1e-3, "Time step must be a float and larger than 1 ms"


    # Extracting solving-parameters
    self.solving_parameters = mpc_parameters["solving_parameters"]


    # Initialize MPC
    self._setup_model()
    self._setup_objective_function()


  def _ode(
        self, 
        x : casadi.SX, 
        u : casadi.SX
      ) -> casadi.SX:
    # Standard MPC - corrected the heave-velocity measurements which was previously thought 
    # to be in Body, and not in NED...

    # This is an ugly and inefficient method, but it works with symbolic variables
    # The use of cos and sin doubles the complexity of the program, and assuming small angles 
    # may be a better method to reduce the program complexity  
    roll = x[5]
    pitch = x[6]

    cos_roll = casadi.cos(roll)
    sin_roll = casadi.sin(roll)

    cos_pitch = casadi.cos(pitch)
    sin_pitch = casadi.sin(pitch)

    # Assumning yaw can be neglected - major assumption!
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
    rotation_matrix_body_to_vehicle = rotation_matrix_y @ rotation_matrix_x
    rotation_matrix_body_to_vehicle = casadi.SX(rotation_matrix_body_to_vehicle)
    rotation_matrix_vehicle_to_body = casadi.transpose(rotation_matrix_body_to_vehicle)

    # Extracting the symbolic states 
    velocity_ned = np.array(
      [
        [x[3]], 
        [x[4]], 
        [u[3]]
      ]
    )
    velocity_body = rotation_matrix_body_to_vehicle @ velocity_ned

    # Derivatives
    # Assuming that the positions are in body 
    d_position_body = velocity_body

    d_horizontal_vel = np.eye(2, 3) @ (
      ((rotation_matrix_vehicle_to_body @ self.g_ned) - self.linear_drag_matrix @ velocity_body) / (self.drone_mass)
    )

    d_rpy = np.array(
      [
        [(self.k_roll * u[0] - roll) / self.tau_roll], 
        [(self.k_pitch * u[1] - pitch) / self.tau_pitch]
      ]
    )
    return casadi.vertcat(*[d_position_body, d_horizontal_vel, d_rpy])


    # # Standard MPC

    # # Extracting the symbolic states 
    # velocity_body = np.array(
    #   [
    #     [x[2]], 
    #     [x[3]], 
    #     [u[2]]
    #   ]
    # )

    # # Ideally, one would like to run Scipy's rotation with the current angles, but this produces
    # # NAN-values when symbolic variables are used in the program 
    # # rotation_matrix_body_to_vehicle = Rotation.from_euler("xyz", np.array([x[6], x[7], x[8]])).as_matrix()

    # # This is an ugly and inefficient method, but it works with symbolic variables
    # # The use of cos and sin doubles the complexity of the program, and assuming small angles 
    # # may be a better method to reduce the program complexity  
    # roll = x[6]
    # pitch = x[7]
    # # yaw = x[8]

    # cos_roll = 1 # casadi.cos(roll)
    # sin_roll = roll # casadi.sin(roll)

    # cos_pitch = 1 # casadi.cos(pitch)
    # sin_pitch = pitch # casadi.sin(pitch)

    # # Assumning yaw can be neglected - major assumption!
    # cos_yaw = 1 # casadi.cos(yaw)
    # sin_yaw = 0 # casadi.sin(yaw)

    # rotation_matrix_x = np.array(
    #   [
    #     [1, 0, 0],
    #     [0, cos_roll, -sin_roll],
    #     [0, sin_roll, cos_roll]
    #   ]
    # )
    # rotation_matrix_y = np.array(
    #   [
    #     [cos_pitch, 0, sin_pitch],
    #     [0, 1, 0],
    #     [-sin_pitch, 0, cos_pitch]
    #   ]
    # )
    # rotation_matrix_z = np.array(
    #   [
    #     [cos_yaw, -sin_yaw, 0],
    #     [sin_yaw, cos_yaw, 0],
    #     [0, 0, 1]
    #   ]
    # )
    # rotation_matrix_body_to_vehicle = rotation_matrix_z @ rotation_matrix_y @ rotation_matrix_x
    # rotation_matrix_body_to_vehicle = casadi.SX(rotation_matrix_body_to_vehicle)
    # rotation_matrix_vehicle_to_body = casadi.transpose(rotation_matrix_body_to_vehicle)

    # # Derivatives
    # # Assuming that the positions are in body 
    # d_position_body = velocity_body
    # # d_position_body = rotation_matrix_body_to_vehicle @ np.array(
    # #   [
    # #     [x[3]], 
    # #     [x[4]], 
    # #     [u[3]]  
    # #   ]
    # # )

    # d_horizontal_vel = np.eye(2, 3) @ (
    #   ((rotation_matrix_vehicle_to_body @ self.g_ned) - self.linear_drag_matrix @ velocity_body) / (self.drone_mass)
    # )
    # d_vertical_vel = 0  # -u[3] # Change in velocity defined in NED, while the thrust is in ENU
    #                     # Unsure if the change of the vertical state should be modelled
    #                     # If not, could reduce the state space by two states - this and yaw

    # d_rpy = np.array(
    #   [
    #     [(self.k_roll * u[0] - x[6]) / self.tau_roll], 
    #     [(self.k_pitch * u[1] - x[7]) / self.tau_pitch], 
    #     [0]#u[2]]
    #   ]
    # )
    # print(*[d_position_body, d_horizontal_vel, d_vertical_vel, d_rpy])
    # return casadi.vertcat(*[d_position_body, d_horizontal_vel, d_vertical_vel, d_rpy])


    # Augmented state space with integral action
    # x = [xi, yi, x, y, z, u, v, phi, theta]
    # pos_body = np.array(
    #   [
    #     [x[2]], 
    #     [x[3]] 
    #   ]
    # )

    # velocity_body = np.array(
    #   [
    #     [x[5]], 
    #     [x[6]],
    #     [u[3]]
    #   ]
    # )

    # roll = x[7]
    # pitch = x[8]

    # cos_roll = 1 
    # sin_roll = roll 

    # cos_pitch = 1 
    # sin_pitch = pitch 

    # # Assumning yaw can be neglected - major assumption!

    # rotation_matrix_x = np.array(
    #   [
    #     [1, 0,          0],
    #     [0, cos_roll,  -sin_roll],
    #     [0, sin_roll,   cos_roll]
    #   ]
    # )
    # rotation_matrix_y = np.array(
    #   [
    #     [cos_pitch,   0,  sin_pitch],
    #     [0,           1,  0],
    #     [-sin_pitch,  0,  cos_pitch]
    #   ]
    # )

    # rotation_matrix_body_to_vehicle = rotation_matrix_y @ rotation_matrix_x
    # rotation_matrix_body_to_vehicle = casadi.SX(rotation_matrix_body_to_vehicle)
    # rotation_matrix_vehicle_to_body = casadi.transpose(rotation_matrix_body_to_vehicle)

    # # Derivatives
    # d_integrated_position_body = pos_body
    # d_position_body = velocity_body
    # d_horizontal_vel = (
    #   np.eye(2, 3) @ ((rotation_matrix_vehicle_to_body @ self.g_ned) - self.linear_drag_matrix @ velocity_body) / (self.drone_mass)
    # )

    # d_rp = np.array(
    #   [
    #     [(self.k_roll * u[0] - roll) / self.tau_roll], 
    #     [(self.k_pitch * u[1] - pitch) / self.tau_pitch]
    #   ]
    # )
    # return casadi.vertcat(*[d_integrated_position_body, d_position_body, d_horizontal_vel, d_rp])



  def _setup_model(self) -> None:
    model_type = "continuous"
    self.model = do_mpc.model.Model(model_type)

    self.x = self.model.set_variable(var_type='_x', var_name='x', shape=(self.nx,1))
    self.u = self.model.set_variable(var_type='_u', var_name='u', shape=(self.nu,1))

    self.model.set_rhs('x', self._ode(self.x, self.u))
    self.model.setup()
  

  def _setup_objective_function(self) -> None:  
    xTQ = casadi.mtimes(casadi.transpose(self.x), casadi.SX(np.diag(self.q_diagonal)))
    xTQx = casadi.mtimes(xTQ, self.x)

    # uTR = casadi.mtimes(casadi.transpose(u), casadi.SX(np.diag(self.r_diagonal)))
    # uTRu = casadi.mtimes(uTR, u)

    self.lagrange_term = xTQx
    self.meyer_term = xTQx
    # self.r_term = uTRu
    self.r_term = self.r_diagonal


  def get_mpc_solver(self) -> do_mpc.controller.MPC:
    mpc_solver = do_mpc.controller.MPC(self.model)

    # https://www.do-mpc.com/en/latest/api/do_mpc.controller.MPC.set_param.html
    collocation_degree = self.solving_parameters["collocation_degree"]
    solver_options = self.solving_parameters["solver_options"]
    solver = solver_options["solver"]
    print_level = solver_options["debug_level"]
    if print_level == 0:
      print_time = 0
    else:
      print_time = 1 # Unsure about the unit
    setup_mpc = {
        'n_horizon': self.prediction_horizon,       # Prediction horizon
        'n_robust': self.robust_horizon,            # Robust horizon for a robust scenario MPC
        'open_loop': False,                         # False will generate an individual cntrol input for each scenario. 
                                                    # True will use the same control input for all scenarios 
        't_step': self.time_step,                   # Time step of the MPC
        'use_terminal_bounds': False,               # Bool indicating if terminal constraints are used
        'collocation_deg': collocation_degree,      # Degree of collocation for continous models - https://www.do-mpc.com/en/latest/theory_orthogonal_collocation.html
        'nl_cons_single_slack': True,               # True will use slacking variables for soft constraints 
        'store_full_solution': False,
        'store_lagr_multiplier': False,
        'nlpsol_opts': {
          'ipopt.linear_solver': solver,            
          'ipopt.print_level': print_level,         
          'ipopt.sb': 'yes', 
          'print_time': print_time
        }
    }
    mpc_solver.set_param(**setup_mpc)

    # Configure objective function:
    mpc_solver.set_objective(lterm=self.lagrange_term, mterm=self.meyer_term)
    mpc_solver.set_rterm(u=np.array(self.r_term)) 

    # State and input bounds - use the config file
    mpc_solver.bounds['lower', '_x', 'x'] = [\
                                              -inf, -inf, -100, \
                                              -4, -4, \
                                              -5*np.pi/180.0, -5*np.pi/180.0
                                            ]
    mpc_solver.bounds['upper', '_x', 'x'] = [\
                                              inf, inf, 0.5, \
                                              4, 4, \
                                              5*np.pi/180.0, 5*np.pi/180.0
                                            ]
    
    mpc_solver.bounds['lower', '_u', 'u'] = [-5*np.pi/180.0, -5*np.pi/180.0, -10*np.pi/180.0, -0.15]
    mpc_solver.bounds['upper', '_u', 'u'] = [5*np.pi/180.0, 5*np.pi/180.0, 10*np.pi/180.0, 0.15]

    mpc_solver.setup()

    return mpc_solver
