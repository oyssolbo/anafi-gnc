
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

import homography

class PoseRecovery():

    def __init__(self, K, camera_offsets):
        self.K = K
        self.camera_offsets = camera_offsets

    def find_homography(self, features_image, features_metric):
        # uv is image pixel location (origin = upper left)
        uv = features_image.copy()
        uv1 = homography.homogenize(uv)

        # xy is image coordinates with origin = center
        xy_homgen = np.linalg.solve(self.K, uv1)
        xy = homography.dehomogenize(xy_homgen)

        # XY are the metric coordinates in the helipad frame
        XY = features_metric[0:2].copy()

        # H = homography.estimate_H_linear(xy, XY)
        H = homography.estimate_H_opencv(xy.T, XY.T)

        return H

    def find_R_t_pnp(self, object_points, image_points):
        distortion_coeffs = np.zeros(5)

        # cv.solvePnP(...) requires object points to be of shape (n_points, 3)
        # and image points to be of shape (n_points, 2)
        object_points = homography.dehomogenize(object_points.copy()).T
        image_points = image_points.T

        _, R_vec, t_vec = cv.solvePnP(object_points, image_points, self.K, distortion_coeffs)

        # Using solvePnPRansac did not make much of a difference, see 2022-5-5/18-8-48
        # _, R_vec, t_vec, inliers = cv.solvePnPRansac(object_points, image_points, self.K, distortion_coeffs)
        # if len(inliers) != 5:
        #     print(f"Only found {len(inliers)} inliers")
        # R_vec2, t_vec2 = cv.solvePnPRefineLM(object_points, image_points, self.K, distortion_coeffs, R_vec, t_vec)
        # print(np.array_equal(t_vec, t_vec2))
        Rt, _ = cv.Rodrigues(R_vec)

        # cameraPosition = -Rt.T @ t_vec
        R = Rt

        t = t_vec.reshape((3,))

        return R, t

    def find_R_t_homography(self, features_image, features_metric, H):
        XY01 = features_metric.copy()

        T1, T2 = homography.decompose_H(H)
        T = homography.choose_decomposition(T1, T2, XY01)

        R = T[:3, :3]
        t = T[:3, 3]

        return R, t

    def optimize_R_t(self, features_image, features_metric, R, t):

        # uv is image pixel location (origin = upper left)
        uv = features_image.copy()
        uv1 = homography.homogenize(uv)

        # xy is image coordinates with origin = center
        xy_homgen = np.linalg.solve(self.K, uv1)
        xy = homography.dehomogenize(xy_homgen)

        # XY are the metric coordinates in the helipad frame
        XY01 = features_metric.copy()

        R, t = homography.estimate_Rt_ls(
                    xy, XY01, R.copy(), t.copy(), self.K, uv, num_iterations=100, debug=False)

        return R, t

    def get_pose_from_R_t(self, R, t):

        # The orientation angles are probably incorrect, but this has not been investigated
        # as only the position is used in the EKF.
        orientation = homography.rotation_matrix2euler_angles(R)*180/np.pi
        pos = t

        pose = np.hstack((pos, orientation))

        return pose

    def camera_to_drone_body_frame(self, R_camera, t_camera):
        # The x and y axis of the camera frame span the image plane, and the z-axis
        # sits perpendicular to this plane out of the camera. The camera frame axes are therefore
        # as follows:
        #   x-axis: Positive right in image plane
        #   y-axis: Positive down in image plane
        #   z-axis: Positive out of the image plane
        # Converting this to the body frame therefore only involved rotating the whole
        # camera frame 90 degrees counter clockwise, andn then adjusting for the
        # camera offsets.

        rz = homography.rotate_z(np.pi/2)[:3, :3]

        R_body = rz @ R_camera


        # angles = homography.rotation_matrix2euler_angles(R_body)
        # print(angles*180/np.pi)
        # rx = homography.rotate_x(angles[0])[:3, :3]
        # ry = homography.rotate_y(angles[1])[:3, :3]

        # R_correction = rx.T @ ry.T # not this see 2022-5-5/18-2-31
        # R_correction = rx @ ry # not this see 2022-5-5/18-4-34
        # R_correction = ry @ rx # not this see 2022-5-5/18-7-11
        # R_correction = ry.T @ rx.T # this is ok, but not that good, see 2022-5-5/17-59-45
        # R_correction = ry.T @ rx.T
        # t_body = rz @ R_correction @ t_camera

        t_body = rz @ t_camera

        t_body += self.camera_offsets

        # Manual offsets TODO: make find these using some least squares or something
        # t_body[0] -= 0.1
        # t_body[1] -= 0.2
        # t_body[2] -= 0.15

        return R_body, t_body

    def evaluate_reprojection(self, image, features_image, features_metric, R_camera, t_camera, H=None, R_LM=None, t_LM=None):

        # uv is image pixel location (origin = upper left)
        uv = features_image.copy()

        # XY are the metric coordinates in the helipad frame
        XY = features_metric[0:2].copy()
        XY1 = homography.homogenize(XY)
        XY01 = features_metric.copy()

        if H is not None:
            # For H
            uv_H = homography.reproject_using_H(self.K, H, XY1)
            err_H = homography.reprojection_error(uv, uv_H)
            print("Reprojection error using H:", err_H)

        # For R,t
        uv_Rt = homography.reproject_using_Rt(self.K, R_camera, t_camera, XY01)
        err_Rt = homography.reprojection_error(uv, uv_Rt)
        print(f"Reprojection error using R and t: {err_Rt}, average: {np.average(err_Rt)}")
        T_camera = homography.create_T_from_Rt(R_camera, t_camera)
        T_body = homography.create_T_from_Rt(*self.camera_to_drone_body_frame(R_camera, t_camera))

        plt.figure()
        plt.imshow(cv.cvtColor(image, cv.COLOR_BGR2RGB))
        plt.scatter(*uv, s=150, marker='o', c='white', edgecolor='black', label="True")
        plt.scatter(*uv_Rt, s=30, marker='o', c='cyan',
                    edgecolors='black', label="Reprojected using camera frame R and t")
        plt_title = "Reprojections of pose"

        if R_LM is not None and t_LM is not None:
            # For R,t from LM
            uv_Rt_LM = homography.reproject_using_Rt(self.K, R_LM, t_LM, XY01)
            err_Rt_LM = homography.reprojection_error(uv, uv_Rt_LM)
            print(f"Reprojection error using R and t from LM: {err_Rt_LM}, average: {np.average(err_Rt_LM)}")

            plt.scatter(*uv_Rt_LM, s=30, marker='o', c='red',
                        edgecolors='black', label="Reprojected using LM")
            plt_title = "Reprojections of pose with and without optimization"

        plt.title(plt_title)
        homography.draw_frame(self.K, T_camera, scale=0.1)
        plt.legend()

        # Plot body frame in 3D relative to the helipad points
        plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot(XY[0,:], XY[1,:], np.zeros(XY.shape[1]), '.', markersize=10) # Draw markers in 3D
        pO = np.linalg.inv(T_body)@np.array([0,0,0,1]) # Compute camera origin
        pX = np.linalg.inv(T_body)@np.array([0.4,0,0,1]) # Compute camera X-axis
        pY = np.linalg.inv(T_body)@np.array([0,0.4,0,1]) # Compute camera Y-axis
        pZ = np.linalg.inv(T_body)@np.array([0,0,0.4,1]) # Compute camera Z-axis
        plt.plot([pO[0], pZ[0]], [pO[1], pZ[1]], [pO[2], pZ[2]], color='blue', linewidth=2) # Draw camera Z-axis
        plt.plot([pO[0], pY[0]], [pO[1], pY[1]], [pO[2], pY[2]], color='green', linewidth=2) # Draw camera Y-axis
        plt.plot([pO[0], pX[0]], [pO[1], pX[1]], [pO[2], pX[2]], color='red', linewidth=2) # Draw camera X-axis
        # plt.plot([0, 0], [0, 0], [0, 0.4], color='blue', linewidth=2) # Draw camera Z-axis
        # plt.plot([0, 0], [0, 0.4], [0, 0], color='green', linewidth=2) # Draw camera Y-axis
        # plt.plot([0, 0.4], [0, 0], [0, 0], color='red', linewidth=2) # Draw camera X-axis
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-0.5, 1.5])
        ax.set_xlabel('X')
        ax.set_zlabel('Z')
        ax.set_ylabel('Y')

        plt.title("Camera pose relative to detected features")
        # plt.show()

def main():
    # Test reprojection using
    import os
    import sys
    import yaml
    import time
    import feature_detector

    shi_tomasi_config = {
        "max_corners" : 13,
        "quality_level" : 0.0001,
        "min_distance" : 1,
        "block_size" : 7,
        "gradient_size" : 17,
        "k" : 0.04,
        "use_harris_detector": True
    }

    hough_circle_config = {
        "bilateral_diameter": 9,
        "dp": 1,
        "gaussian_kernel": 5,
        "max_radius": 500,
        "median_kernel": 11,
        "method": 3,
        "min_dist": 1000,
        "min_radius": 50,
        "param1": 40,
        "param2": 70,
        "use_bilateral_blur": True,
        "use_gaussian_blur": True,
        "use_median_blur": True
    }

    detector = feature_detector.FeatureDetector(shi_tomasi_config, hough_circle_config)

    script_dir = os.path.dirname(os.path.realpath(__file__))

    try:
        with open(f"{script_dir}/../../config/tcv_config.yaml") as f:
            config = yaml.safe_load(f)
    except Exception as e:
        print(f"Failed to load config: {e}")
        sys.exit()

    features_metric = np.loadtxt(
            f"{script_dir}/../../{config['feature_dists_metric']['path']}"
    )
    # features_metric = np.loadtxt("../../data/helipad_dists_origin_center_enu_metric.txt")

    K = np.loadtxt(
        f"{script_dir}/../../data/camera_matrix.txt"
    )

    camera_offsets = np.array([0.09, 0, 0])

    pose_recoverer = PoseRecovery(K, camera_offsets)

    img = cv.imread("test_images/real/frame0055.jpg")
    mask = detector.create_helipad_mask(img, show_masked_img=False)
    corners = detector.find_corners_shi_tomasi(img, mask)
    # detector.show_corners_found(img, corners, color="red")
    features_image = detector.find_arrow_and_H(corners, features_metric)
    # detector.show_known_points(img, features_image)

    # Not using this homograpghy based solution anymore
    # H = pose_recoverer.find_homography(features_image, features_metric)
    # start = time.time()
    # R, t = pose_recoverer.find_R_t_homography(features_image, features_metric, H)
    # print(f"Recovering R and t took {time.time() - start} seconds")
    # start = time.time()
    # R_LM, t_LM = pose_recoverer.optimize_R_t(features_image, features_metric, R, t)
    # print(f"Optmizing R and t took {time.time() - start} seconds")
    # pose_raw = pose_recoverer.get_pose_from_R_t(R, t)
    # pose = pose_recoverer.get_pose_from_R_t(R_LM, t_LM)
    # print(f"Raw pose from R and t Pos: {pose_raw[:3]} Orientation: {pose_raw[3:]}")
    # print(f"Optimized pose: Pos: {pose[:3]} Orientation: {pose[3:]}")

    R_pnp_camera, t_pnp_camera = pose_recoverer.find_R_t_pnp(features_metric, features_image)
    R_pnp_body, t_pnp_body = pose_recoverer.camera_to_drone_body_frame(R_pnp_camera, t_pnp_camera)
    pose_pnp_body = pose_recoverer.get_pose_from_R_t(R_pnp_body, t_pnp_body)
    pose_recoverer.evaluate_reprojection(img, features_image, features_metric, R_pnp_camera, t_pnp_camera)

    print(f"PnP pose from R and t Pos: {pose_pnp_body[:3]} Orientation: {pose_pnp_body[3:]}")


    # cv.waitKey()
    plt.show()

if __name__ == "__main__":
    main()