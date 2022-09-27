#!/usr/bin/env python3

import os
import argparse
import functools
import numpy as np
import pandas as pd

import seaborn as sns

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

class Plotter():

    def __init__(self):
        sns.set_theme()
        # LaTex must be installed for this to work
        # sudo apt-get install dvipng texlive-latex-extra texlive-fonts-recommended cm-super

        plt.rcParams['text.usetex'] = True
        plt.rc('text.latex', preamble=r'\usepackage{bm}')

    def match_two_dataseries_one_to_one(self, data1: np.ndarray, data2: np.ndarray):

        if data1.shape[0] > data2.shape[0]:
            dense = data1
            dense_prefix = "data1"
            sparse = data2
            sparse_prefix = "data2"
        else:
            dense = data2
            dense_prefix = "data2"
            sparse = data1
            sparse_prefix = "data1"

        dense_df = pd.DataFrame()
        dense_df["time"] = dense[:,0]
        for i in range(1, dense.shape[1]):
            dense_df[f"{dense_prefix}_{i-1}"] = dense[:,i]

        sparse_df = pd.DataFrame()
        sparse_df["time"] = sparse[:,0]
        for i in range(1, sparse.shape[1]):
            sparse_df[f"{sparse_prefix}_{i-1}"] = sparse[:,i]

        merged = pd.merge_asof(sparse_df, dense_df, on="time", allow_exact_matches=True, direction="nearest")

        timestamps = merged["time"].to_numpy()

        data1_out = []
        for i in range(data1.shape[1] - 1):
            data1_out.append(merged[f"data1_{i}"].to_numpy())
        data1_out = np.array(data1_out).T

        data2_out = []
        for i in range(data2.shape[1] - 1):
            data2_out.append(merged[f"data2_{i}"].to_numpy())
        data2_out = np.array(data2_out).T

        return timestamps, data1_out, data2_out

    def sync_multiple_data_series_based_on_timestamps(self, data: list):

        data_frames = []

        for i, dataseries in enumerate(data):
            df = pd.DataFrame()
            df["time"] = dataseries[:,0]
            for j in range(1, dataseries.shape[1]):
                df[f"data_{i}_{j-1}"] = dataseries[:,j]
            data_frames.append(df)

        df_merged = functools.reduce(lambda left,right: pd.merge(left, right, on="time", how="outer"), data_frames)

        data_out = []

        for i in range(len(data)):
            data_i = []
            data_i.append(df_merged["time"])

            num_columns = data[i].shape[1] - 1 # not including time
            for j in range(num_columns):
                data_i.append(df_merged[f"data_{i}_{j}"].to_numpy())
            data_i = np.array(data_i).T

            data_out.append(data_i)

        return data_out

    def calculate_rmse_and_plot(self, timestamps, data1, data2, suptitle="", data1_label="", data2_label="",
        xlabels=[], ylabels=[], plot_std_devs=False, std_devs=[], legend_size=10
    ):
        if data1.shape == data2.shape:
            rmse_all = np.sqrt(np.mean((data1 - data2)**2))
            rmse_per_column = []
            for i in range(data1.shape[1]):
                rmse_per_column.append(np.sqrt(np.mean((data1[:,i] - data2[:,i])**2)))

            print("="*10, f"RMSE calculations ({suptitle})", "="*10)
            print(f"Total RMSE ({data1.shape[1]} axis): {rmse_all:.4f}\n")
            for i, rmse in enumerate(rmse_per_column):
                print(f"Column {i}: {rmse:.4f}")

        fig, ax = plt.subplots(data1.shape[1], 1)
        fig.suptitle(suptitle)

        for i in range(ax.shape[0]):
            if i == ax.shape[0] - 1:
                ax[i].set_xlabel(xlabels[i])
            else:
                ax[i].set_xticklabels([])


            ax[i].set_ylabel(ylabels[i])
            ax[i].plot(timestamps - timestamps[0], data1[:,i], label=data1_label, linewidth=2)
            ax[i].plot(timestamps - timestamps[0], data2[:,i], label=data2_label, linewidth=2)

            if plot_std_devs:
                ax[i].fill_between(timestamps - timestamps[0], y1=data1[:,i] - std_devs[i], y2=data1[:,i] + std_devs[i], alpha=.5, label=r"$1\sigma$")
                # ax[i].plot(timestamps - timestamps[0], std_devs[i], c="red", label="std")

            if i == ax.shape[0] - 1: # only print legend on last figure
                ax[i].legend(loc="lower center", prop={'size': legend_size})

        fig.align_ylabels(ax)
        plt.savefig("matched_data.png", dpi=300, bbox_inches="tight", pad_inches=0)

    def plot_multiple_data_series(self, data: list, numplots: int, suptitle: str,
        legends: list, xlabels: list, ylabels: list, use_scatter: list, legend_size=10
    ):
        fig, ax = plt.subplots(numplots, 1)
        fig.suptitle(suptitle)
        try:
            for i in range(ax.shape[0]):
                if i == ax.shape[0] - 1:
                    ax[i].set_xlabel(xlabels[i])
                else:
                    ax[i].set_xticklabels([])
                ax[i].set_ylabel(ylabels[i])
                # ax[i].yaxis.set_label_position("right")
                for j in range(len(data)):
                    if use_scatter[j]:
                        ax[i].plot(data[j][:,0] - data[j][0,0], data[j][:,i+1], marker=".", label=legends[j])
                    else:
                        ax[i].plot(data[j][:,0] - data[j][0,0], data[j][:,i+1], label=legends[j])
                if i == ax.shape[0] - 1: # only print legend on last figure
                    ax[i].legend(loc="best", prop={'size': legend_size})
        except AttributeError: # means there is only one plot
            ax.set_ylabel(ylabels[0])
            ax.set_xlabel(xlabels[0])
            for j in range(len(data)):
                    if use_scatter[j]:
                        ax.scatter(data[j][:,0] - data[j][0,0], data[j][:,1], s=4, label=legends[j])
                    else:
                        ax.plot(data[j][:,0] - data[j][0,0], data[j][:,1], label=legends[j])
            ax.legend(loc="upper right", prop={'size': 10})
        fig.align_ylabels(ax)
        # plt.subplots_adjust(top=0.8)
        print(fig.get_size_inches())
        plt.savefig(f"multipe_data_series.png", dpi=300, bbox_inches="tight", pad_inches=0)

    def plot_ned_positions_3d(self, drone_pos:np.ndarray, helipad_pos: np.ndarray, plot_title="", orientation: tuple=None):
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        # ax.set_xlim(-2, 2)
        # ax.set_ylim(-2, 2)

        ax.plot3D(helipad_pos[:,1], helipad_pos[:,2], helipad_pos[:,3], label=r"$\bm{p}_{h}^n$")
        ax.plot3D(drone_pos[:,1], drone_pos[:,2], drone_pos[:,3], label=r"$\bm{p}_{d}^n$", linestyle="dashdot")

        if orientation is not None:
            ax.view_init(elev=orientation[0], azim=orientation[1])

        ax.legend()
        ax.set_title(plot_title)
        plt.savefig("3d_plot.png", dpi=300, bbox_inches="tight", pad_inches=0)


def main():
    parser = argparse.ArgumentParser(description="Visualize data.")
    parser.add_argument("data_dir", metavar="data_dir", type=str, help="Base directory of data")

    args = parser.parse_args()

    plotter = Plotter()

    script_dir = os.path.dirname(os.path.realpath(__file__))
    data_dir = f"{script_dir}/../../../../../out/{args.data_dir}"

    # Load data
    # gt_data = np.loadtxt(f"{data_dir}/ground_truths/helipad_pose_body_frame.txt", skiprows=2)
    # gt_data_drone_pose = np.loadtxt(f"{data_dir}/ground_truths/drone_pose_helipad_frame.txt", skiprows=2)
    anafi_raw_data = np.loadtxt(f"{data_dir}/estimates/anafi_raw_data.txt", skiprows=1)

    # # Calculate accuracy of DNNCV
    # ts, dnncv_pos, gt_pos = plotter.match_two_dataseries_one_to_one(dnncv_data, gt_data)
    # plotter.calculate_rmse_and_plot(
    #     ts,
    #     dnncv_pos[:,:3],
    #     gt_pos[:,:3],
    #     suptitle="DNNCV pos estimate vs. ground truth",
    #     data1_label="DNNCV",
    #     data2_label="GT",
    #     ylabels=["x[m]", "y[m]", "z[m]"],
    #     xlabels=["t [sec]", "t [sec]", "t [sec]"]
    # )

    # # Calculate accuracy of EKF
    # ts, ekf_pos, gt_pos = plotter.match_two_dataseries_one_to_one(ekfpos_data, gt_data)
    # plotter.calculate_rmse_and_plot(
    #     ts,
    #     ekf_pos[:,:3],
    #     gt_pos[:,:3],
    #     suptitle="EKF pos estimate vs. ground truth",
    #     data1_label="EKF",
    #     data2_label="GT",
    #     ylabels=["x[m]", "y[m]", "z[m]"],
    #     xlabels=["t [sec]", "t [sec]", "t [sec]"],
    #     plot_std_devs=True,
    #     std_devs = [np.sqrt(ekf_pos[:,3]), np.sqrt(ekf_pos[:,7]), np.sqrt(ekf_pos[:,11])]
    # )

    # ###################
    # # Only estimates (EKF, DNNCV, TCV)
    # ###################

    # tcv_data = np.loadtxt(f"{data_dir}/estimates/tcv_pose.txt", skiprows=1)
    # dnncv_data = np.loadtxt(f"{data_dir}/estimates/dnn_cv_position.txt", skiprows=1)
    # ekf_data = np.loadtxt(f"{data_dir}/estimates/ekf_position.txt", skiprows=1)

    # title = ""

    # synced_gt_dnncv_ekf_data = plotter.sync_multiple_data_series_based_on_timestamps([ekf_data, dnncv_data, tcv_data])
    # plotter.plot_multiple_data_series(
    #     synced_gt_dnncv_ekf_data, 3, title,
    #     ["KF", "DNNCV", "TCV"], ["t [sec]", "t [sec]", "t [sec]"], ["x[m]", "y[m]", "z[m]"],
    #     [False, False, True]
    # )

    # ###################
    # # TCV
    # ###################

    # tcv_data = np.loadtxt(f"{data_dir}/estimates/tcv_pose.txt", skiprows=1)
    # gt_data = np.loadtxt(f"{data_dir}/ground_truths/helipad_pose_body_frame.txt", skiprows=2)

    # if args.data_dir == "2022-5-25/22-58-16":
    #     title = r"TCV estimate vs. ground truth position $\bm p_h^b$" "\n" "Flight pattern: Square with yaw rotation at $\sim$60 sec"
    #     title = ""
    # else:
    #     title = "Ground truth vs. TCV position"

    # # Compare TCV estimate to ground truth
    # synced_gt_tcv_data = plotter.sync_multiple_data_series_based_on_timestamps([gt_data, tcv_data])
    # plotter.plot_multiple_data_series(
    #     synced_gt_tcv_data, 3, title,
    #     ["GT", "TCV"], ["t [sec]", "t [sec]", "t [sec]"], ["x[m]", "y[m]", "z[m]"],
    #     [False, False]
    # )

    # # Calculate accuracy of TCV
    # ts, gt_pos, tcv_pos = plotter.match_two_dataseries_one_to_one(gt_data, tcv_data)
    # plotter.calculate_rmse_and_plot(
    #     ts,
    #     gt_pos[:,:3],
    #     tcv_pos[:,:3],
    #     suptitle="TCV pos estimate vs. ground truth",
    #     data1_label="GT",
    #     data2_label="TCV",
    #     ylabels=["x[m]", "y[m]", "z[m]"],
    #     xlabels=["t [sec]", "t [sec]", "t [sec]"],
    # )

    # ###################
    # # DNNCV
    # ###################

    # dnncv_data = np.loadtxt(f"{data_dir}/estimates/dnn_cv_position.txt", skiprows=1)
    # gt_data = np.loadtxt(f"{data_dir}/ground_truths/helipad_pose_body_frame.txt", skiprows=2)

    # if args.data_dir == "2022-5-25/23-6-43":
    #     title = r"DNNCV estimate vs. ground truth position $\bm p_h^b$" "\n" "Flight pattern: Square with yaw rotation at $\sim$60 sec"
    #     title = ""
    # else:
    #     title = "Ground truth vs. DNNCV position"

    # # Compare DNNCV estimate to ground truth
    # synced_gt_dnncv_data = plotter.sync_multiple_data_series_based_on_timestamps([gt_data, dnncv_data])
    # plotter.plot_multiple_data_series(
    #     synced_gt_dnncv_data, 3, title,
    #     ["GT", "DNNCV"], ["t [sec]", "t [sec]", "t [sec]"], ["x[m]", "y[m]", "z[m]"],
    #     [False, False]
    # )

    # # Calculate accuracy of DNNCV
    # ts, dnncv_pos, gt_pos = plotter.match_two_dataseries_one_to_one(dnncv_data, gt_data)
    # plotter.calculate_rmse_and_plot(
    #     ts,
    #     dnncv_pos[:,:3],
    #     gt_pos[:,:3],
    #     suptitle="DNNCV pos estimate vs. ground truth",
    #     data1_label="DNNCV",
    #     data2_label="GT",
    #     ylabels=["x[m]", "y[m]", "z[m]"],
    #     xlabels=["t [sec]", "t [sec]", "t [sec]"],
    # )

    # ###################
    # # EKF
    # ###################

    # ekf_data = np.loadtxt(f"{data_dir}/estimates/ekf_position.txt", skiprows=1)
    # gt_data = np.loadtxt(f"{data_dir}/ground_truths/helipad_pose_body_frame.txt", skiprows=2)

    # if args.data_dir == "2022-5-26/13-49-36":
    #     title = "Perception output - Drone landing on discretely moving platform\nEnvironment: REAL - Guidance law: PID"
    # elif args.data_dir == "2022-5-26/13-55-56":
    #     title = "Perception output - Drone landing on continuously moving platform\nEnvironment: REAL - Guidance law: PID"
    # elif args.data_dir == "2022-5-25/23-16-1":
    #     title = r"CV model KF estimate vs. ground truth position $\bm p_h^b$" "\n" "Flight pattern: Square with yaw rotation at $\sim$60 sec"
    #     title = ""
    # elif args.data_dir == "2022-5-25/23-48-35":
    #     title = r"Complete KF estimate vs. ground truth position $\bm p_h^b$" "\n" "Flight pattern: Square with yaw rotation at $\sim$60 sec"
    #     title = ""
    # else:
    #     title = "Position - GT vs. KF (all measurements)"

    # # Compare EKF estimate to ground truth
    # synced_gt_ekf_data = plotter.sync_multiple_data_series_based_on_timestamps([gt_data, ekf_data])
    # plotter.plot_multiple_data_series(
    #     synced_gt_ekf_data, 3, title,
    #     ["GT", "KF"], ["t [sec]", "t [sec]", "t [sec]"], ["x[m]", "y[m]", "z[m]"],
    #     [False, False], legend_size=10
    # )

    # # Calculate accuracy of EKF
    # ts, ekf_pos, gt_pos = plotter.match_two_dataseries_one_to_one(ekf_data, gt_data)
    # plotter.calculate_rmse_and_plot(
    #     ts,
    #     ekf_pos[:,:3],
    #     gt_pos[:,:3],
    #     suptitle=title,
    #     data1_label="KF",
    #     data2_label="GT",
    #     ylabels=["x[m]", "y[m]", "z[m]"],
    #     xlabels=["t [sec]", "t [sec]", "t [sec]"],
    #     plot_std_devs=True,
    #     std_devs = [np.sqrt(ekf_pos[:,3]), np.sqrt(ekf_pos[:,7]), np.sqrt(ekf_pos[:,11])],
    #     legend_size=10
    # )

    # # ###################
    # # # GT, EKF, DNNCV and TCV all in one
    # # ###################

    tcv_data = np.loadtxt(f"{data_dir}/estimates/tcv_pose.txt", skiprows=1)
    dnncv_data = np.loadtxt(f"{data_dir}/estimates/dnn_cv_position.txt", skiprows=1)
    ekf_data = np.loadtxt(f"{data_dir}/estimates/ekf_position.txt", skiprows=1)
    gt_data = np.loadtxt(f"{data_dir}/ground_truths/helipad_pose_body_frame.txt", skiprows=2)

    if args.data_dir == "2022-5-26/13-49-36":
        title = "Perception output - Drone landing on discretely moving platform\nEnvironment: REAL - Guidance law: PID"
        title = ""
    elif args.data_dir == "2022-5-26/13-55-56":
        title = "Perception output - Drone landing on continuously moving platform\nEnvironment: REAL - Guidance law: PID"
        title = ""
    elif args.data_dir == "2022-5-25/23-48-35":
        title = r"Complete KF estimate vs. ground truth position $\bm p_h^b$" "\n" "Flight pattern: Square with yaw rotation at $\sim$60 sec"
    else:
        title = "Position - GT vs. EKF vs. DNNCV raw vs. TCV raw"

    # # Compare DNNCV and EKF estimate to ground truth
    synced_gt_dnncv_ekf_data = plotter.sync_multiple_data_series_based_on_timestamps([gt_data, ekf_data, dnncv_data, tcv_data])
    legend = ["GT", "KF", "DNNCV", "TCV"]
    plotter.plot_multiple_data_series(
        synced_gt_dnncv_ekf_data, 3, title,
        legend, ["t [sec]", "t [sec]", "t [sec]"], ["x[m]", "y[m]", "z[m]"],
        [False, False, False, False], legend_size=8
    )

    # ###################
    # # GT only
    # ###################

    # gt_data = np.loadtxt(f"{data_dir}/ground_truths/helipad_pose_body_frame.txt", skiprows=2)

    # # title = "Flight patthern used to evaluate perception systems" "\n" r"Ground truth position $\bm p_h^b$"
    # title = ""

    # # # Compare DNNCV and EKF estimate to ground truth
    # legend = ["GT"]
    # plotter.plot_multiple_data_series(
    #     [gt_data], 3, title,
    #     legend, ["t [sec]", "t [sec]", "t [sec]"], ["x[m]", "y[m]", "z[m]"],
    #     [False]
    # )

    # plt.subplots_adjust(top=0.8)

    # # ###################
    # # # GT vs. measured velocity
    # # ###################
    # anafi_raw_data = np.loadtxt(f"{data_dir}/estimates/anafi_raw_data.txt", skiprows=1)
    # gt_odom_data = np.loadtxt(f"{data_dir}/ground_truths/drone_velocity_body_frame_and_attitude.txt", skiprows=2)


    # synced_gt_telemetry_data = plotter.sync_multiple_data_series_based_on_timestamps([gt_odom_data, anafi_raw_data])
    # plotter.plot_multiple_data_series(
    #     synced_gt_telemetry_data, 3, "GT vs. telemetry velocity",
    #     ["GT", "TEL"], ["t [sec]", "t [sec]", "t [sec]"], ["x[m]", "y[m]", "z[m]"],
    #     [False, False]
    # )

    # Plot heading angles
    # synched_heading_data = plotter.sync_multiple_data_series_based_on_timestamps([gt_data_drone_pose, anafi_raw_data])

    # plotter.plot_multiple_data_series(
    #     [
    #         np.array([synced_gt_tcv_data[0][:,0], synced_gt_tcv_data[0][:,6]]).T,
    #         np.array([synced_gt_tcv_data[1][:,0], synced_gt_tcv_data[1][:,6]]).T
    #     ],
    #      1, "Heading - GT vs TCV", ["GT", "tcv"],  ["t [sec]"], ["heading [deg]"], [False, False]
    # )

    # ###################
    # # 3D NED position
    # ###################
    drone_pose_ned_gt = np.loadtxt(f"{data_dir}/ground_truths/drone_pose_ned.txt", skiprows=2)
    helipad_pose_ned_gt = np.loadtxt(f"{data_dir}/ground_truths/helipad_pose.txt", skiprows=2)

    if args.data_dir == "2022-5-26/13-49-36":
        title = "3D view - Drone landing on discretely moving platform\nEnvironment: REAL - Guidance law: PID"
        title = ""
        orientation = (12, -127)
    elif args.data_dir == "2022-5-26/13-55-56":
        title = "3D view - Drone landing on continuously moving platform\nEnvironment: REAL - Guidance law: PID"
        title = ""
        orientation = (11, -110)
    else:
        title = "Add title here"
        orientation = None

    plotter.plot_ned_positions_3d(drone_pose_ned_gt, helipad_pose_ned_gt, plot_title=title, orientation=orientation)

    plt.show()

if __name__ == "__main__":
    main()