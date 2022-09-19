import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv

def homogenize(v):
    """
    Append a row of 1's to the bottom of the column vector v (shape (n,) ).
    Works for vectors or matrices of shape (n,N).
    """
    if len(v.shape) == 1:
        u = np.append(v, 1)
    else:
        n = v.shape[1]
        u = np.vstack((v, np.ones(n)))
    return u

def dehomogenize(v):
    """
    Dehomogenize v by dividing each element by last element. Works with vectors
    (shape (n,) ) or matrices (shape (n,N) ).
    Args:
        v: homogeneous vector/matrix
    Return:
        u: dehomogenized vector without 1 as last element in each column.
    """
    return homogeneous_normalize(v)[:-1]

def homogeneous_normalize(v):
    return v / v[-1]

def estimate_H_opencv(xy, XY):
    H, _ = cv.findHomography(XY, xy)

    return H

def estimate_H_linear(xy_, XY_):
    """
    Estimate the homography between planar points in xy and XY. xy (lower-case)
    are the image coordinates with origin in the center of the image (i.e.
    result from xy = Kinv * uv). XY are the coplanar points in the world.
    Args:
        xy: image coordinates with origin = center, shape (2, n) or (3, n)
            (latter if homogeneous coordinates)
        XY: coplanar points in the world, shape (2, n) or (3, n) (latter if
            homogeneous coorindates)
    Returns:
        H: homography between xy and XY s.t. xy = H * XY
    """
    if xy_.shape[0] == 3:
        xy = dehomogenize(xy_)
    else:
        xy = xy_.copy()

    if XY_.shape[0] == 3:
        XY = dehomogenize(XY_)
    else:
        XY = XY_.copy()

    n = XY.shape[1]

    A = np.zeros((2*n, 9))
    for i in range(n):
        xi, yi = xy[0, i], xy[1, i]
        Xi, Yi = XY[0, i], XY[1, i]

        Ai = np.array([
            [Xi, Yi, 1, 0, 0, 0, -Xi*xi, -Yi*xi, -xi],
            [0, 0, 0, Xi, Yi, 1, -Xi*yi, -Yi*yi, -yi]
        ])

        A[2*i:2*i+2, :] = Ai

    U, S, VT = np.linalg.svd(A)

    H = VT[-1].reshape((3, 3))  # last column of V.T

    return H

def closest_rotation_matrix(Q):
    U, S, VT = np.linalg.svd(Q)
    R = U @ VT
    return R

def decompose_H(H, best_approx=True):
    # best_approx: if True, use Zhangs method of approximating by minimizing
    # the frobenius norm.
    k = np.linalg.norm(H[:, 0])
    r1 = H[:, 0] / k
    r2 = H[:, 1] / k
    r3_pos = np.cross(r1, r2)
    r3_neg = np.cross(-r1, -r2)
    t = H[:, 2] / k

    R_pos = np.hstack((r1[:, None], r2[:, None], r3_pos[:, None]))
    R_neg = np.hstack((-r1[:, None], -r2[:, None], r3_neg[:, None]))

    if best_approx:
        R_pos = closest_rotation_matrix(R_pos)
        R_neg = closest_rotation_matrix(R_neg)

    # index [:, None] gives extra dimensions so vectors are (3,1) and not (3,)
    T1 = np.hstack((R_pos, t[:, None]))
    T1 = np.vstack((T1, np.array([0, 0, 0, 1])))

    T2 = np.hstack((R_neg, -t[:, None]))
    T2 = np.vstack((T2, np.array([0, 0, 0, 1])))

    return T1, T2

def choose_decomposition(T1, T2, XYZ1):
    # here, XYZ1 = XY01 because Z = 0
    c1 = T1 @ XYZ1
    c2 = T2 @ XYZ1

    if np.all(c1[2, :] >= 0):
        T = T1
    elif np.all(c2[2, :] >= 0):
        T = T2
    else:
        raise ValueError("Neither T1 nor T2 gives physically plausable pose.")

    return T

def reproject_using_H(K, H, XY1):
    """
    Reprojects the points in XY1 using the homography H.
    Returns:
        Dehomogenized coordinates uv in pixel coords.
    """
    uv_homgen = K @ H @ XY1
    uv = dehomogenize(uv_homgen)
    return uv

def reproject_using_Rt(K, R, t, XYZ1):
    """
    Reprojects the points in XYZ1 using the rigid-body transformation R and t.
    Returns:
        Dehomogenized coordinates uv in pixel coords.
    """
    # projection matrix
    P = np.hstack((
        np.eye(3), np.zeros((3, 1))
    ))

    T = create_T_from_Rt(R, t)
    uv_homgen = K @ P @ T @ XYZ1
    uv = dehomogenize(uv_homgen)
    return uv


def create_T_from_Rt(R, t):
    T = np.hstack((R, t[:, None]))
    T = np.vstack((T, np.array([0, 0, 0, 1])))
    return T

def levenberg_marquardt(
    residualsfun,
    p0,
    num_iterations=100,
    termination_tolerance=0.001,
    finite_difference_epsilon=1e-5,
    debug=False
):
    p = p0.copy()
    mu = None
    iterations = 0

    def objfun(p):
        r = residualsfun(p)
        return r.T @ r

    prev = 0
    while iterations < num_iterations:
        if debug:
            print("Iteration number", iterations)
        J = finite_difference(residualsfun, p, finite_difference_epsilon)

        JTJ = J.T @ J
        r = residualsfun(p)
        JTr = J.T @ r

        # Initialize mu if not already initialized
        if mu is None:
            mu = 1e-3 * np.max(np.diag(JTJ))
            if debug:
                print("  LM: Initialized mu to:", mu)

        # Update rule for damping parameter mu
        delta = np.ones((JTJ.shape[0],))
        while np.linalg.norm(delta) > 1e-10:
            delta = np.linalg.solve(JTJ + mu*np.eye(JTJ.shape[0]), -JTr)
            if objfun(p + delta) < objfun(p):
                mu /= 3
                if debug:
                    print("  LM: Reduced mu:", mu)
                break
            else:
                mu *= 2
                if debug:
                    print("  LM: Increased mu:", mu)
        else:
            # will run only if the while-termination criterion is met
            if debug:
                print("  LM: Delta small:", np.linalg.norm(delta))

        # Update estimate
        p = p + delta
        iterations += 1
        if debug:
            print(f"-> E(p) = {objfun(p)}")

        curr = objfun(p)
        if np.abs(curr - prev) <= termination_tolerance:
            if debug:
                print("  LM: Stop because below termination tolerance.")
            break
        prev = curr

    return p, iterations

def finite_difference(f, x, epsilon):
    """
    Args:
        f: function, returns n x 1, shape = (n,)
        x: vector, m x 1, shape = (m,)
        epsilon: finite difference epsilon
    Return:
        J = df/fx, n x m
    """
    y = f(x)
    n = y.shape[0]

    m = x.shape[0]
    J = np.zeros((n, m))
    for i in range(m):
        e = np.zeros((m,))
        e[i] = epsilon
        Ji = f(x + e) - f(x - e)
        Ji /= 2*epsilon
        J[:, i] = Ji

    return J

def reprojection_error(uv, uv_pred):
    # Calculates the reprojection error based on the 2-norm of the difference
    # between the true and predicted points.
    # Returns mean, min, max

    # xy_pred = H @ XY1
    # uv_from_H_homgen = K @ xy_pred
    # uv_predicted = uv_from_H_homgen[0:2] / uv_from_H_homgen[2]

    norms = np.linalg.norm(uv - uv_pred, axis=0)
    # return norms.mean(), norms.min(), norms.max()
    return norms

def estimate_Rt_ls(xy, XYZ1, R0, t0, K, uv, num_iterations=100, debug=False):

    n = xy.shape[1]

    # Inputs:
    # xy, XY, [R0, t0] from H (DLT), K, uv

    # parameterize
    # R(p) = R(p1)R(p2)R(p3) R0
    # v = [p1, p2, p3, t1, t2, t3]
    # estimate v
    # return T = [R(p), t] (homogeneous)
    xy1 = np.vstack((xy, np.ones((1, n))))

    def residualfun(v):
        R = rotate_x(v[0])[:3, :3] @ \
            rotate_y(v[1])[:3, :3] @ \
            rotate_z(v[2])[:3, :3] @ R0
        t = v[3:]
        uv_pred = reproject_using_Rt(K, R, t, XYZ1)
        return reprojection_error(uv, uv_pred)

    v = np.hstack((np.zeros((3,)), t0))  # initial guess
    v, iterations_used = levenberg_marquardt(
        residualfun, v, num_iterations=num_iterations, debug=debug)

    print(f"LM used {iterations_used} iterations")

    R = rotate_x(v[0])[:3, :3] @ \
        rotate_y(v[1])[:3, :3] @ \
        rotate_z(v[2])[:3, :3] @ R0
    t = v[3:]

    return R, t

def rotate_x(radians):
    c = np.cos(radians)
    s = np.sin(radians)
    return np.array([[1, 0, 0, 0],
                        [0, c, -s, 0],
                        [0, s, c, 0],
                        [0, 0, 0, 1]])


def rotate_y(radians):
    c = np.cos(radians)
    s = np.sin(radians)
    return np.array([[c, 0, s, 0],
                     [0, 1, 0, 0],
                     [-s, 0, c, 0],
                     [0, 0, 0, 1]])


def rotate_z(radians):
    c = np.cos(radians)
    s = np.sin(radians)
    return np.array([[c, -s, 0, 0],
                     [s, c, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


def translate(x, y, z):
    return np.array([[1, 0, 0, x],
                     [0, 1, 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]])

def rotation_matrix2euler_angles(R) :

    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = np.arctan2(R[2,1] , R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else :
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

def project(K, X):
    """
    Computes the pinhole projection of a (3 or 4)xN array X using
    the camera intrinsic matrix K. Returns the pixel coordinates
    as an array of size 2xN.
    """
    uvw = K@X[:3, :]
    uvw /= uvw[2, :]
    return uvw[:2, :]

def draw_frame(K, T, scale=1):
    """
    Visualize the coordinate frame axes of the 4x4 object-to-camera
    matrix T using the 3x3 intrinsic matrix K.
    Control the length of the axes by specifying the scale argument.
    """
    X = T @ np.array([
        [0, scale, 0, 0],
        [0, 0, scale, 0],
        [0, 0, 0, scale],
        [1, 1, 1, 1]])
    u, v = project(K, X)
    plt.plot([u[0], u[1]], [v[0], v[1]], color='red')  # X-axis
    plt.plot([u[0], u[2]], [v[0], v[2]], color='green')  # Y-axis
    plt.plot([u[0], u[3]], [v[0], v[3]], color='blue')  # Z-axis