
import os
import yaml
import glob
import time
import tqdm
import cv2 as cv
import numpy as np
import pandas as pd
import corner_detector_optimizer
import circle_detector_optimizer
from scipy.spatial import distance
from sklearn.neighbors import KNeighborsClassifier

def get_corner_labels_from_csv(filename: str, frame_id: str):
    # Returns [[x1, y1],
    #          [x2, y2],
    #          .........
    #          [x12, y12]]
    return pd.read_csv(filename, index_col=0).loc[frame_id, :].to_numpy().reshape((13,2))

def corner_estimation_error(pred: np.ndarray, gt: np.ndarray, verbose=False):
    classes = np.arange(len(pred))

    knn = KNeighborsClassifier(n_neighbors=1)
    knn.fit(pred, y=classes)
    gt_ind = knn.predict(gt)

    if len(np.unique(gt_ind)) != len(gt):
        # print("Could not match prediction and ground truth points one to one")
        return None

    gt_sorted = gt[np.argsort(gt_ind)]
    pred_sorted = pred

    errors = np.diagonal(distance.cdist(pred_sorted, gt_sorted))

    if verbose:
        print("Matching the following points ([pred_x, pred_y] <-> [gt_x, gt_y])")
        for (pred_coord, gt_coord, error) in zip(pred_sorted, gt_sorted, errors):
            print(f"{pred_coord} <-> {gt_coord} error: {error:.3f}")

    mean_error = np.mean(errors)

    if mean_error >= 5:
        return None

    return mean_error

def load_images(misdetections_only=False):
    script_dir = os.path.dirname(os.path.realpath(__file__))
    if misdetections_only:
        frame_ids = [
            'frame0024.jpg', 'frame0025.jpg', 'frame0047.jpg', 'frame0121.jpg', 'frame0192.jpg',
            'frame0228.jpg', 'frame0229.jpg', 'frame0231.jpg', 'frame0258.jpg'
        ]
        filenames = [f"{script_dir}/../test_images/real/{frame_id}" for frame_id in frame_ids]
    else:
        filenames = glob.glob(f"{script_dir}/../test_images/real/*.jpg")

    images = [(cv.imread(file), file) for file in sorted(filenames)]

    return images

def get_mask(img, xyr, show_masked_img=False):
    (x, y, r) = xyr
    circle_mask = np.zeros((720,1280), np.uint8)
    cv.circle(circle_mask, (x,y), int(r * 1.40), (255, 0, 255), cv.FILLED)

    if show_masked_img:
        img_masked = cv.bitwise_and(img, img, mask=circle_mask)
        cv.imshow("Segmented image", img_masked)

    return circle_mask

def show_corners_found(img, corners, color, mask=None):
        image = np.copy(img)

        if mask is not None:
            image = cv.bitwise_and(image, image, mask=mask)

        if color == "red":
            c = (0,0,255)
        elif color == "blue":
            c = (255,0,0)

        for i in range(corners.shape[0]):
            center = (int(corners[i,0]), int(corners[i,1]))

            text_face = cv.FONT_HERSHEY_DUPLEX
            text_scale = 0.5
            # text_scale = 0.7
            text_thickness = 1
            text = f"{i}"
            text_offset = 10

            text_size, _ = cv.getTextSize(text, text_face, text_scale, text_thickness)
            text_origin = (
                int(center[0] - text_size[0] / 2) + text_offset,
                int(center[1] + text_size[1] / 2) - text_offset
            )

            # cv.circle(image, center, 6, c, cv.FILLED)
            cv.circle(image, center, 4, c, cv.FILLED)
            cv.putText(image, text, text_origin, text_face, text_scale, (127,255,127), text_thickness, cv.LINE_AA)

        cv.imshow("Detected corners", image)

def evaluate_corner_detector():

    script_dir = os.path.dirname(os.path.realpath(__file__))

    with open(f"{script_dir}/../../../data/shi_tomasi_params.yaml") as f:
        corner_detector_params = yaml.safe_load(f)

    corner_detector = corner_detector_optimizer.CornerDetector(
        quality_level=corner_detector_params["quality_level"],
        min_distance=corner_detector_params["min_distance"],
        block_size=corner_detector_params["block_size"],
        gradient_size=corner_detector_params["gradient_size"],
        use_harris_detector=corner_detector_params["use_harris_detector"],
        k=corner_detector_params["k"],
        max_corners=corner_detector_params["max_corners"]
    )

    with open(f"{script_dir}/../../../data/hough_circle_params.yaml") as f:
        hough_circle_params = yaml.safe_load(f)

    circle_detector = circle_detector_optimizer.CircleDetector(
        method=hough_circle_params["method"],
        dp=hough_circle_params["dp"],
        param1=hough_circle_params["param1"],
        param2=hough_circle_params["param2"],
        min_radius=hough_circle_params["min_radius"],
        max_radius=hough_circle_params["max_radius"],
        use_gaussian_blur=hough_circle_params["use_gaussian_blur"],
        gaussian_kernel=hough_circle_params["gaussian_kernel"],
        use_median_blur=hough_circle_params["use_median_blur"],
        median_kernel=hough_circle_params["median_kernel"],
        use_bilateral_blur=hough_circle_params["use_bilateral_blur"],
        bilateral_diameter=hough_circle_params["bilateral_diameter"],
    )

    images = load_images(misdetections_only=False)
    visualize = False

    errors = []
    durations = []
    misdetected_images = []

    for (img, filename) in tqdm.tqdm(images):
        frame_id = os.path.basename(filename)
        gt_corner_params = get_corner_labels_from_csv(f"{filename[:-13]}/corner_labels.csv", frame_id)

        xyr = circle_detector.predict([img.copy().flatten()])[0]
        mask = get_mask(img, xyr)

        start_time = time.time()
        corners = corner_detector.predict([np.hstack((img.copy().flatten(), mask.flatten()))])[0]
        duration = time.time() - start_time
        durations.append(duration)
        corners = corners.reshape(corners.shape[0], 2)
        error = corner_estimation_error(corners, gt_corner_params, verbose=False)

        if error is None:
            misdetected_images.append(frame_id)
        else:
            errors.append(error)

        if visualize:
            show_corners_found(img, corners, "red", mask=mask)
            cv.waitKey()


    print(f"Misdetected images: {len(misdetected_images)}/{len(images)}")
    print(misdetected_images)
    print(10 * "=")
    print(f"Mean prediction error (all images where found): {np.mean(errors)}")
    print(f"Detection time: Average: {np.mean(durations):.4f} Median: {np.median(durations):.4f} Max: {np.max(durations):.4f}")
    print(10 * "=")

if __name__ == "__main__":
    evaluate_corner_detector()