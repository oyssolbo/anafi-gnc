
import os
import time
import glob
import cv2 as cv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial import distance
from sklearn.neighbors import KNeighborsClassifier

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

def circle_estimation_error(pred: np.ndarray, gt: np.ndarray, verbose=False):
    error_origin = np.linalg.norm(pred[:2] - gt[:2])
    error_radius = np.abs(pred[2] - gt[2])

    if verbose:
        print(f"Error origin: \t\t{error_origin:.2f} \t(pred: {(pred[0], pred[1])} gt: {(gt[0], gt[1])})")
        print(f"Error radius (abs): \t{error_radius:.2f} \t(pred: {pred[2]} gt: {gt[2]})")

    if error_radius/gt[2] < 0.2:
        return 0
    else:
        return 1

def show_image_matplotlib(img):
    try: # fails if image is grayscale, then just show it
        img = cv.cvtColor(img.copy(), cv.COLOR_BGR2RGB)
    except:
        pass
    plt.imshow(img),plt.show()

def show_image_opencv(img):
    cv.imshow("Output", img)
    cv.waitKey(1)

def blur_image(img):
    blur = cv.medianBlur(img.copy(), 11)
    blur = cv.GaussianBlur(blur,(5,5),0)
    blur = cv.bilateralFilter(blur,9,75,75)

    return blur

def blur_image_gaussian(img):
    blur = cv.GaussianBlur(img.copy(),(5,5),0)
    return blur

def blur_image_edges(img):
    # Get this to work somehow

    if len(img.shape) == 2: # image is grayscale
        h, w = img.shape
    elif len(img.shape) == 3: # image is color
        h, w, _ = img.shape

    mask = np.zeros(img.shape)

    mask[:, :20] = 1 # left edge
    mask[:, w-20:] = 1 # right edge
    mask[:20, :] = 1 # top edge
    mask[h-20:, :] = 1 # bottom edge

    img_blurred = blur_image(img.copy())
    out = img.copy()
    out[mask>0] = img_blurred[mask>0]
    # show_image_matplotlib(out)
    return out



def show_corners_found(img, corners, color, use_matplotlib=True):
        image = np.copy(img)

        if color == "red":
            c = (0,0,255)
        elif color == "blue":
            c = (255,0,0)

        for i in range(corners.shape[0]):
            center = (int(corners[i,0]), int(corners[i,1]))

            text_face = cv.FONT_HERSHEY_DUPLEX
            text_scale = 0.5
            text_thickness = 1
            text = f"{i}"
            text_offset = 10

            text_size, _ = cv.getTextSize(text, text_face, text_scale, text_thickness)
            text_origin = (
                int(center[0] - text_size[0] / 2) + text_offset,
                int(center[1] + text_size[1] / 2) - text_offset
            )

            cv.circle(image, center, 4, c, cv.FILLED)
            cv.putText(image, text, text_origin, text_face, text_scale, (127,255,127), text_thickness, cv.LINE_AA)

        # cv.imshow("Detected corners", image)
        # plt.imshow(image),plt.show()
        if use_matplotlib:
            show_image_matplotlib(image)
        else:
            show_image_opencv(image)

def find_corners_fast(img, mask):

        fast_feature_dector = cv.FastFeatureDetector_create(threshold=30)

        # print( "Threshold: {}".format(fast_feature_dector.getThreshold()) )
        # print( "nonmaxSuppression:{}".format(fast_feature_dector.getNonmaxSuppression()) )
        # print( "neighborhood: {}".format(fast_feature_dector.getType()) )

        key_points = fast_feature_dector.detect(img, mask=mask)
        # print(help(fast_feature_dector.detect))

        corners = np.array([key_points[idx].pt for idx in range(0, len(key_points))])
        corners = corners.reshape(corners.shape[0], 2)
        # corners = np.array([key_points[idx].pt for idx in range(0, len(key_points))]).reshape(-1, 1, 2)

        return corners

def find_corners_shi_tomasi(img, mask):

    img = cv.cvtColor(img.copy(), cv.COLOR_BGR2GRAY)

    max_corners = 13
    quality_level = 0.01
    min_distance = 10
    block_size = 9
    gradient_size = 3
    k = 0.04

    # New parameters found from grid search
    quality_level = 0.001
    block_size = 5
    gradient_size = 9
    min_distance = 1

    # Parameters from newest grid search
    block_size = 7
    gradient_size = 17
    k = 0.04
    max_corners = 13
    min_distance = 1
    quality_level = 0.0001
    use_harris_detector = True

    start_time = time.time()

    corners = cv.goodFeaturesToTrack(img, max_corners, quality_level,
        min_distance, mask=mask, blockSize=block_size,
        gradientSize=gradient_size, useHarrisDetector=use_harris_detector, k=k
    )

    # print(f"Shi-Tomasi corner detector used {time.time() - start_time:.3f} sec")

    if corners is not None and corners.shape[0] == 13:
        return corners.reshape(corners.shape[0], 2)
    else:
        return np.array([])

def hough_circle(img, use_matplotlib=True):

    blurred_img = blur_image(img)

    gray = cv.cvtColor(blurred_img, cv.COLOR_BGR2GRAY)

    rows = gray.shape[0]

    method = cv.HOUGH_GRADIENT
    dp = 1
    min_dist = 1000
    param1 = 50
    param2 = 50
    min_radius = 10
    max_radius = 700

    # Parameters from grid search test
    param1 = 40
    param2 = 70
    min_radius = 50
    max_radius = 500

    start_time = time.time()

    circles = cv.HoughCircles(gray, method=method, dp=dp, minDist=min_dist,
                        param1=param1, param2=param2,
                        minRadius=min_radius, maxRadius=max_radius)

    # print(f"Hough circle used {time.time() - start_time:.3f} sec")

    output = img.copy()

    if circles is not None and len(circles) == 1:
        # r_largest = 0
        # center_largest = None
        # circles = np.uint16(np.around(circles))
        # for i in circles[0, :]:
        #     center = (i[0], i[1])
        #     radius = i[2]
        #     print(radius)

        #     if radius > r_largest:
        #         r_largest = radius
        #         center_largest = center

        # circle_mask = np.zeros((720,1280), np.uint8)
        # cv.circle(circle_mask, center_largest, int(r_largest * 1.01), (255, 0, 255), cv.FILLED)

        # helipad = cv.bitwise_and(img,img, mask=circle_mask)

        # helipad_gray = cv.cvtColor(helipad, cv.COLOR_BGR2GRAY)
        # blur = cv.medianBlur(helipad_gray, 11)

        # blur = cv.GaussianBlur(blur,(5,5),0)
        # blur = cv.bilateralFilter(blur,9,75,75)
        # output = helipad_gray
        # plt.imshow(output),plt.show()

        # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")

        # Create circle mask
        (x, y, r) = circles[0]
        circle_mask = np.zeros((720,1280), np.uint8)
        cv.circle(circle_mask, (x,y), int(r * 1.40), (255, 0, 255), cv.FILLED)
        img_masked = cv.bitwise_and(img, img, mask=circle_mask)

        # loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv.circle(output, (x, y), r, (0, 255, 0), 4)
            cv.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

        # print(f"Image noise: {estimate_noise(img_masked)}")
        # # show the output image
        # if use_matplotlib:
        #     show_image_matplotlib(img_masked)
        # else:
        #     show_image_opencv(img_masked)

        return img_masked, circle_mask, (x,y,r)
    else:
        # show_image_opencv(img)
        return img, np.ones((720, 1280)), (0,0,0)



def match_features_orb(img1, img2):
    orb = cv.ORB_create()

    # find the keypoints and descriptors with ORB
    kp1, des1 = orb.detectAndCompute(img1,None)
    kp2, des2 = orb.detectAndCompute(img2,None)

    # create BFMatcher object
    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
    # Match descriptors.
    matches = bf.match(des1,des2)
    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance)
    # Draw first 10 matches.
    img3 = cv.drawMatches(img1,kp1,img2,kp2,matches,None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    plt.imshow(img3),plt.show()

def find_corners_sift(img, mask):
    sift = cv.SIFT_create(nfeatures=13)

    key_points = sift.detect(img, mask=mask)

    corners = np.array([key_points[idx].pt for idx in range(0, len(key_points))])
    corners = corners.reshape(corners.shape[0], 2)

    return corners

def match_features_sift(img1, img2):
    # Initiate SIFT detector
    sift = cv.SIFT_create()
    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)
    # BFMatcher with default params
    bf = cv.BFMatcher()
    matches = bf.knnMatch(des1,des2,k=2)
    # Apply ratio test
    good = []
    for m,n in matches:
        if m.distance < 0.75*n.distance:
            good.append([m])
    # cv.drawMatchesKnn expects list of lists as matches.
    img3 = cv.drawMatchesKnn(img1,kp1,img2,kp2,good,None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    plt.imshow(img3),plt.show()

def find_gradient(img):
    gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    laplacian = cv.Laplacian(gray_img, cv.CV_32F)
    # plt.imshow(laplacian, cmap="gray"),plt.show()
    print(np.mean(laplacian))
    cv.imshow("Gradient", laplacian)
    cv.waitKey(5000)

def estimate_noise(I):
    import math
    import scipy.signal
    I = cv.cvtColor(I, cv.COLOR_BGR2GRAY)
    H, W = I.shape

    M = [[1, -2, 1],
        [-2, 4, -2],
        [1, -2, 1]]

    sigma = np.sum(np.sum(np.absolute(scipy.signal.convolve2d(I, M))))
    sigma = sigma * math.sqrt(0.5 * math.pi) / (6 * (W-2) * (H-2))

    return sigma

def run_pipeline_single_image(img, show_corners=True, use_matplot_lib=False):
    img = blur_image_gaussian(img)
    img_masked, circle_mask, _ = hough_circle(img, use_matplotlib=use_matplot_lib)
    corners = find_corners_shi_tomasi(img, circle_mask)
    if show_corners:
        show_corners_found(img_masked, corners, "red", use_matplotlib=use_matplot_lib)

    return corners

def run_cropping_only(img, show_crop=False):
    img = blur_image_gaussian(img)
    try:
        img_masked, circle_mask, circle_params = hough_circle(img)
        out_img = img_masked
    except TypeError:
        out_img = img
        circle_params = np.zeros(3)

    if show_crop:
        show_image_opencv(out_img)
    return np.array(circle_params)

def run_pipeline_all_images():
    images = [(cv.imread(file), file) for file in glob.glob("test_images/real/*.jpg")]

    images_not_found_on = []
    hough_circle_run_times = []
    corner_detector_time_used = []

    for (img, filename) in images:
        img = blur_image_gaussian(img)
        try:
            start_time = time.time()
            img_masked, circle_mask, _ = hough_circle(img, use_matplotlib=False)
            time_used = time.time() - start_time
            hough_circle_run_times.append(time_used)
            print(f"Hough circle used: {time_used:.4f} sec for image {filename}")
            start_time = time.time()
            corners = find_corners_shi_tomasi(img, circle_mask)
            # corners = find_corners_fast(img, circle_mask)
            time_used = time.time() - start_time
            corner_detector_time_used.append(time_used)
            show_corners_found(img_masked, corners, "red", use_matplotlib=False)
        except TypeError:
            show_image_opencv(img)
            images_not_found_on.append((img, filename))

        # time.sleep(0.1)

    print(f"Mean Hough circle run time: {np.mean(np.array(hough_circle_run_times))}")
    print(f"Mean corner detector run time: {np.mean(np.array(corner_detector_time_used))}")
    print(f"Could not find circles in {len(images_not_found_on)} images:\n{sorted([filename for (_, filename) in images_not_found_on])}")

def get_corner_labels_from_csv(filename: str, frame_id: str):
    # Returns [[x1, y1],
    #          [x2, y2],
    #          .........
    #          [x12, y12]]
    return pd.read_csv(filename, index_col=0).loc[frame_id, :].to_numpy().reshape((13,2))

def get_circle_labels_from_csv(filename: str, frame_id: str):
    # Returns [x,y,r]
    return pd.read_csv(filename, index_col=0).loc[frame_id, :].to_numpy()

def evaluate_corner_detector():
    # Evaluate corner detector
    # Load images

    images = [(cv.imread(file), file) for file in sorted(glob.glob("test_images/real/*.jpg"))]

    images = [(cv.imread("test_images/real/frame0153.jpg"), "test_images/real/frame0153.jpg")]

    errors = []
    misdetections = 0
    misdetected_images = []

    # For testing only the ones where it is misdetected
    # misdetected_images = ['test_images/real/frame0004.jpg', 'test_images/real/frame0025.jpg', 'test_images/real/frame0028.jpg',
    #     'test_images/real/frame0073.jpg', 'test_images/real/frame0121.jpg', 'test_images/real/frame0132.jpg',
    #     'test_images/real/frame0145.jpg', 'test_images/real/frame0148.jpg', 'test_images/real/frame0205.jpg',
    #     'test_images/real/frame0225.jpg', 'test_images/real/frame0228.jpg', 'test_images/real/frame0229.jpg',
    #     'test_images/real/frame0231.jpg', 'test_images/real/frame0249.jpg', 'test_images/real/frame0258.jpg'
    # ]
    # images = [(cv.imread(file), file) for file in sorted(misdetected_images)]


    for (img, filename) in images:

        frame_id = os.path.basename(filename)
        # if frame_id == "frame0004.jpg":
        #     continue

        gt_labels = get_corner_labels_from_csv(f"{filename[:-13]}/corner_labels.csv", frame_id)
        corners = run_pipeline_single_image(img, show_corners=True)
        if corners.size != 0:
            error = corner_estimation_error(corners, gt_labels, verbose=False)
        else:
            error = None

        if error is None:
            print(f"Could not find correct corners in image: {filename}")
            misdetections += 1
            misdetected_images.append(filename)
        else:
                errors.append(error)

        # print(f"Mean prediction error: {error}")
        cv.waitKey(0)

    print(f"Misdetected images: {misdetections}/{len(images)}")
    print(misdetected_images)
    print(f"Mean prediction error (all images where found): {np.mean(errors)}")

def evaluate_circle_detector():
    # Load images
    images = [(cv.imread(file), file) for file in sorted(glob.glob("test_images/real/*.jpg"))]

    errors = []

    for (img, filename) in images:
        frame_id = os.path.basename(filename)
        gt_circle_params = get_circle_labels_from_csv(f"{filename[:-13]}/circle_labels.csv", frame_id)
        circle_params = run_cropping_only(img, show_crop=True)
        error = circle_estimation_error(circle_params, gt_circle_params, verbose=False)
        if error:
            print(f"Could not find correct circle in image: {filename}")
        errors.append(error)
        cv.waitKey(0)

    print(f"Misdetected images: {np.count_nonzero(errors)}/{len(errors)}")

if __name__ == "__main__":
    # evaluate_circle_detector()
    evaluate_corner_detector()