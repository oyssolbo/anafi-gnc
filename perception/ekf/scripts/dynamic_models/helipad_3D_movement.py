
import numpy as np

class DynamicModel():

    _Q = np.diag([1, 1, 1, 1, 1, 1, 1])
    n = 7

    def f(self, x, u, dt):
        """
        Calculate zero-noise transition from current x

        x = [drone_position (x,y,z),
             drone heading relative helipad (psi),
             helipad_velocity (x,y,z)]

        u = [drone_velocity_body(x_dot,y_dot,z_dot),
             heading_change_of_drone (psi_dot)]

        """

        x_next = np.zeros_like(x, dtype=float)
        
        x_next[0] = x[0] + dt*(u[0]*np.cos(x[3]) + u[1]*np.sin(x[3])) - dt*x[4]
        x_next[1] = x[1] + dt*(-u[0]*np.sin(x[3]) + u[1]*np.cos(x[3])) - dt*x[5]
        x_next[2] = x[2] + dt*u[2] - dt*x[6]
        x_next[3] = x[3] + dt*u[3]
        x_next[4] = x[4]
        x_next[5] = x[5]
        x_next[6] = x[6]

        return x_next.copy()

    def F(self, x, u, dt):
        """
        Calculate transition function Jacobian
        """

        F = np.array([
            [1, 0, 0, dt*(-u[0]*np.sin(u[3]) + u[1]*np.cos(u[3])), -dt, 0, 0],
            [0, 1, 0, dt*(-u[0]*np.cos(u[3]) - u[1]*np.sin(u[3])), 0, -dt, 0],
            [0, 0, 1, 0, 0, 0, -dt],
            [0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 1]
        ])

        return F.copy()

    def Q(self, x, dt):
        return self._Q.copy()