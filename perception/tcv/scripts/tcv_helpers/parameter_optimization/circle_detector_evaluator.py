
import os
import yaml
import glob
import time
import tqdm
import cv2 as cv
import numpy as np
import pandas as pd
import circle_detector_optimizer

def get_circle_labels_from_csv(filename: str, frame_id: str):
    # Returns [x,y,r]
    return pd.read_csv(filename, index_col=0).loc[frame_id, :].to_numpy()

def circle_misdetection(pred: np.ndarray, gt: np.ndarray, verbose=False):
    error_origin = np.linalg.norm(pred[:2] - gt[:2])
    error_radius = np.abs(pred[2] - gt[2])

    if verbose:
        print(f"Error origin: \t\t{error_origin:.2f} \t(pred: {(pred[0], pred[1])} gt: {(gt[0], gt[1])})")
        print(f"Error radius (abs): \t{error_radius:.2f} \t(pred: {pred[2]} gt: {gt[2]})")

    if error_radius/gt[2] < 0.2:
        return 0
    else:
        return 1

def show_segmented_image(img, xyr):
    (x, y, r) = xyr
    circle_mask = np.zeros((720,1280), np.uint8)
    cv.circle(circle_mask, (x,y), int(r * 1.40), (255, 0, 255), cv.FILLED)
    img_masked = cv.bitwise_and(img, img, mask=circle_mask)

    cv.imshow("Segmented image", img_masked)

def load_images(slow_only=False, misdetections_only=False):
    script_dir = os.path.dirname(os.path.realpath(__file__))
    if misdetections_only:
        frame_ids = ['frame0024.jpg', 'frame0025.jpg', 'frame0047.jpg', 'frame0192.jpg', 'frame0229.jpg', 'frame0230.jpg', 'frame0231.jpg']
        filenames = [f"{script_dir}/../test_images/real/{frame_id}" for frame_id in frame_ids]
    elif slow_only:
        frame_ids = ['frame0130.jpg', 'frame0131.jpg', 'frame0139.jpg', 'frame0140.jpg', 'frame0141.jpg',
        'frame0142.jpg', 'frame0143.jpg', 'frame0145.jpg', 'frame0190.jpg', 'frame0203.jpg', 'frame0204.jpg']
        filenames = [f"{script_dir}/../test_images/real/{frame_id}" for frame_id in frame_ids]
    else:
        filenames = glob.glob(f"{script_dir}/../test_images/real/*.jpg")

    images = [(cv.imread(file), file) for file in sorted(filenames)]

    return images

def evaluate_circle_detector():

    script_dir = os.path.dirname(os.path.realpath(__file__))

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

    images = load_images(slow_only=False, misdetections_only=False)

    errors = []
    mask_durations = []
    slow_images = []
    misdetected_images = []

    for (img, filename) in tqdm.tqdm(images):
        frame_id = os.path.basename(filename)
        gt_circle_params = get_circle_labels_from_csv(f"{filename[:-13]}/circle_labels.csv", frame_id)

        mask_start_time = time.time()
        xyr = circle_detector.predict([img.copy().flatten()])[0]
        mask_duration = time.time() - mask_start_time
        mask_durations.append(mask_duration)

        error = circle_misdetection(xyr, gt_circle_params, verbose=False)
        if error:
            misdetected_images.append(frame_id)
        errors.append(error)

        if mask_duration >= 0.30:
            slow_images.append((frame_id, f"{mask_duration:.4f} sec"))

        # show_segmented_image(img, xyr)
        # cv.waitKey(0)

    print(f"Misdetected images: {len(misdetected_images)}/{len(images)}")
    print(misdetected_images)
    print(10 * "=")
    print(f"Mask time: Average: {np.mean(mask_durations):.4f} Median: {np.median(mask_durations):.4f} Max: {np.max(mask_durations):.4f}")
    print(10 * "=")
    print(f"Images that took longer than 300 ms to compute: {len(slow_images)}/{len(images)}")
    print(slow_images)

if __name__ == "__main__":
    evaluate_circle_detector()