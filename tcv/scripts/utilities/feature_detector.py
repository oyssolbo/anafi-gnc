
import cv2 as cv
import numpy as np
from parameter_optimization.circle_detector_optimizer import CircleDetector
from parameter_optimization.corner_detector_optimizer import CornerDetector

HSV_SIM_GREEN = [120, 100, 30]

HUE_MARGIN = 15
SAT_MARGIN = 15
VAL_MARGIN = 15

HUE_LOW_GREEN = max(0, HSV_SIM_GREEN[0] - HUE_MARGIN)
HUE_HIGH_GREEN = min(360, HSV_SIM_GREEN[0] + HUE_MARGIN)

SAT_LOW_GREEN = 85
SAT_HIGH_GREEN = 100

VAL_LOW_GREEN = 15
VAL_HIGH_GREEN = 60

class FeatureDetector():

    def __init__(self, shi_tomasi_config, hough_circle_config):

        self._corner_detector = CornerDetector(
            quality_level=shi_tomasi_config["quality_level"],
            min_distance=shi_tomasi_config["min_distance"],
            block_size=shi_tomasi_config["block_size"],
            gradient_size=shi_tomasi_config["gradient_size"],
            use_harris_detector=shi_tomasi_config["use_harris_detector"],
            k=shi_tomasi_config["k"],
            max_corners=shi_tomasi_config["max_corners"]
        )

        self._circle_detector = CircleDetector(
            method=hough_circle_config["method"],
            dp=hough_circle_config["dp"],
            param1=hough_circle_config["param1"],
            param2=hough_circle_config["param2"],
            min_radius=hough_circle_config["min_radius"],
            max_radius=hough_circle_config["max_radius"],
            use_gaussian_blur=hough_circle_config["use_gaussian_blur"],
            gaussian_kernel=hough_circle_config["gaussian_kernel"],
            use_median_blur=hough_circle_config["use_median_blur"],
            median_kernel=hough_circle_config["median_kernel"],
            use_bilateral_blur=hough_circle_config["use_bilateral_blur"],
            bilateral_diameter=hough_circle_config["bilateral_diameter"],
        )

        self.fast_feature_dector = cv.FastFeatureDetector_create()

    def create_helipad_mask(self, img, show_masked_img=False):

        xyr = self._circle_detector.predict([img.copy().flatten()])[0]

        circle_mask = np.zeros((720,1280), np.uint8)
        if np.count_nonzero(xyr) != 0:
            cv.circle(circle_mask, (xyr[0], xyr[1]), int(xyr[2] * 1.40), (255, 0, 255), cv.FILLED)

        if show_masked_img:
            img_masked = cv.bitwise_and(img, img, mask=circle_mask)
            cv.imshow("Masked image", img_masked)
            cv.waitKey(1)

        return circle_mask

    def preprocess_image(self, img, segment=False):

        if segment:
            # # Reduce the noise to avoid false circle detection
            blur = cv.medianBlur(img, 5)

            # blur = cv.GaussianBlur(blur,(5,5),0)
            # blur = cv.bilateralFilter(blur,9,75,75)

            hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

            low_green = (36, 25, 25)
            high_green = (70, 255,255)

            # mask = cv.inRange(hsv, low_green, high_green)
            mask = cv.inRange(hsv, low_green, high_green)
            segmented = cv.bitwise_and(img,img, mask= mask)

            cv.imshow("segmented", segmented)

            find_circle = False
            if find_circle:

            # Make image grayscale
                gray = cv.cvtColor(segmented, cv.COLOR_BGR2GRAY)

                rows = gray.shape[0]
                circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 8,
                                    param1=100, param2=30,
                                    minRadius=200, maxRadius=500)

                if circles is not None:
                    r_largest = 0
                    center_largest = None
                    circles = np.uint16(np.around(circles))
                    for i in circles[0, :]:
                        center = (i[0], i[1])
                        radius = i[2]
                        print(radius)

                        if radius > r_largest:
                            r_largest = radius
                            center_largest = center

                    circle_mask = np.zeros((720,1280), np.uint8)
                    cv.circle(circle_mask, center_largest, int(r_largest * 1.01), (255, 0, 255), cv.FILLED)

                    helipad = cv.bitwise_and(img,img, mask=circle_mask)

                    helipad_gray = cv.cvtColor(helipad, cv.COLOR_BGR2GRAY)
                    blur = cv.medianBlur(helipad_gray, 11)

                    blur = cv.GaussianBlur(blur,(5,5),0)
                    blur = cv.bilateralFilter(blur,9,75,75)
                    output = helipad_gray
            else:
                output = cv.cvtColor(segmented, cv.COLOR_BGR2GRAY)

        else:
            output = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        return output

    def find_corners_fast(self, img):
        key_points = self.fast_feature_dector.detect(img, None)

        corners = np.array([key_points[idx].pt for idx in range(0, len(key_points))]).reshape(-1, 1, 2)

        return corners

    def find_corners_shi_tomasi(self, img, mask):

        corners = self._corner_detector.predict([np.hstack((img.copy().flatten(), mask.flatten()))])[0]

        if np.count_nonzero(corners) != 0:
            return corners
        else:
            return np.array([])

    def find_arrow_and_H(self, corners, helipad_dists_metric):

        if corners.shape[0] != 13:
            return None

        # Compute distances between all corners
        dists = np.zeros((13,13))
        for i, xy_i in enumerate(corners):
            for j, xy_j in enumerate(corners):
                dists[i,j] = cv.norm(xy_i, xy_j)

        # Find the index of the 4 largest distances (4 since both distances will be
        # counted twice in the dists matrix (i.e. distance from point x to y and y to x
        # will be the same and both the largest))
        inds = np.unravel_index(np.argsort(-dists, axis=None)[:4], dists.shape)

        # The arrow index appears twice in each set of indecies while the corners
        # only appear once, so it can be found by finding the index most frequent in
        # any of the inds arrays
        arrow_idx = np.bincount(inds[0]).argmax()

        # The indecies not associated to the arrow (in no particular order)
        H_corner_idns = inds[0][np.not_equal(inds[0], arrow_idx)]

        arrow_coords = corners[arrow_idx]
        corner_0_idx = H_corner_idns[0]
        corner_0_coords = corners[corner_0_idx]
        corner_1_idx = H_corner_idns[1]
        corner_1_coords = corners[corner_1_idx]

        # Remove outliers

        # Check that distance between arrow and both corners is the same
        if not np.allclose(dists[arrow_idx, corner_0_idx], dists[arrow_idx, corner_1_idx], atol=3):
            return None

        # Check that ratio between distances in the image and in real life are the same
        dist_ratio_px = dists[arrow_idx, corner_0_idx] / dists[corner_0_idx, corner_1_idx]
        dist_ratio_m = cv.norm(helipad_dists_metric[:3,0], helipad_dists_metric[:3,4]) \
                     / cv.norm(helipad_dists_metric[:3,0], helipad_dists_metric[:3,1])
        if not np.allclose(dist_ratio_px, dist_ratio_m, atol=3):
            return None

        # Determine which corner is the left corner of the H and which is the right

        # Case when arrow is above both corners
        if arrow_coords[1] < corner_0_coords[1] and arrow_coords[1] < corner_1_coords[1]:
            # Left will be whichever has the smallest x value
            if corner_0_coords[0] < corner_1_coords[0]:
                h_bottom_left_coords = corner_0_coords
                h_bottom_right_coords = corner_1_coords
            else:
                h_bottom_right_coords = corner_0_coords
                h_bottom_left_coords = corner_1_coords

        # Case when arrow is below both corners
        elif arrow_coords[1] > corner_0_coords[1] and arrow_coords[1] > corner_1_coords[1]:
            # Left will be whichever has the largest x value
            if corner_0_coords[0] > corner_1_coords[0]:
                h_bottom_left_coords = corner_0_coords
                h_bottom_right_coords = corner_1_coords
            else:
                h_bottom_right_coords = corner_0_coords
                h_bottom_left_coords = corner_1_coords

        # Case when arrow is to the right of both corners
        elif arrow_coords[0] > corner_0_coords[0] and arrow_coords[0] > corner_1_coords[0]:
            # Left will be whichever has the smallest y value
            if corner_0_coords[1] < corner_1_coords[1]:
                h_bottom_left_coords = corner_0_coords
                h_bottom_right_coords = corner_1_coords
            else:
                h_bottom_right_coords = corner_0_coords
                h_bottom_left_coords = corner_1_coords

        # Case when arrow is to the left of both corners
        elif arrow_coords[0] < corner_0_coords[0] and arrow_coords[0] < corner_1_coords[0]:
            # Left will be whichever has the largest y value
            if corner_0_coords[1] > corner_1_coords[1]:
                h_bottom_left_coords = corner_0_coords
                h_bottom_right_coords = corner_1_coords
            else:
                h_bottom_right_coords = corner_0_coords
                h_bottom_left_coords = corner_1_coords

        else:
            print("Unable to determine corners")
            return None

        if np.all(h_bottom_left_coords == corner_0_coords):
            h_bottom_left_idx = corner_0_idx
            h_bottom_right_idx = corner_1_idx
        else:
            h_bottom_right_idx = corner_0_idx
            h_bottom_left_idx = corner_1_idx

        # Find ratio between pixels and meters using the distance between the
        # two lower corners
        pixels_per_meter = dists[h_bottom_left_idx, h_bottom_right_idx] \
                         / cv.norm(helipad_dists_metric[:3,0], helipad_dists_metric[:3,1])

        # Now that we know the two bottom corners, we can find the two top corners

        # Find pixel distance between lower left and top right, and then find
        # this distance in the distances matrix to find the index of the top
        # right corner
        dist_lower_left_top_right_px = pixels_per_meter * cv.norm(
            helipad_dists_metric[:3,0],
            helipad_dists_metric[:3,2]
        )
        h_top_right_idx = np.argmin(
            np.abs(dists[h_bottom_left_idx] - dist_lower_left_top_right_px)
        )
        h_top_right_coords = corners[h_top_right_idx]

        # Same procedure for top left
        # dist_lower_left_top_left_px = pixels_per_meter * cv.norm(
        #     helipad_dists_metric[:3,0],
        #     helipad_dists_metric[:3,3]
        # )
        # h_top_left_idx = np.argmin(
        #     np.abs(dists[h_bottom_left_idx] - dist_lower_left_top_left_px)
        # )
        # h_top_left_coords = corners[h_top_left_idx]

        dist_lower_right_top_left_px = pixels_per_meter * cv.norm(
            helipad_dists_metric[:3,1],
            helipad_dists_metric[:3,3]
        )
        h_top_left_idx = np.argmin(
            np.abs(dists[h_bottom_right_idx] - dist_lower_right_top_left_px)
        )
        h_top_left_coords = corners[h_top_left_idx]

        # More consistency checks

        # Check that distance between bottom left and top left is equal to distance
        # between bottom right and top right
        if not np.allclose(dists[h_bottom_left_idx, h_top_left_idx], dists[h_bottom_right_idx, h_top_right_idx], atol=3):
            return None

        ret = np.hstack((
            h_bottom_left_coords.reshape(-1,1),
            h_bottom_right_coords.reshape(-1,1),
            h_top_right_coords.reshape(-1,1),
            h_top_left_coords.reshape(-1,1),
            arrow_coords.reshape(-1,1),
        ))

        return ret

    def show_known_points(self, img, features, mask=None):
        image = np.copy(img)

        if mask is not None:
            image = cv.bitwise_and(image, image, mask=mask)

        text_face = cv.FONT_HERSHEY_DUPLEX
        # text_scale = 0.7
        text_scale = 0.5
        text_thickness = 1
        text_offset = 10
        # text_offset = 20
        texts = ["Lower left", "Lower right", "Top right", "Top left", "Arrow"]

        for (point, text) in zip(features.T, texts):
            center = (int(point[0]), int(point[1]))
            text_size, _ = cv.getTextSize(text, text_face, text_scale, text_thickness)
            text_origin = (
                int(center[0] - text_size[0] / 2) + text_offset,
                int(center[1] + text_size[1] / 2) - text_offset
            )

            cv.circle(image, center, 4, (0,0,255), cv.FILLED)
            # cv.circle(image, center, 8, (0,0,255), cv.FILLED)
            # cv.putText(image, text, text_origin, text_face, text_scale, (0,0,0), text_thickness*3, cv.LINE_AA)
            cv.putText(image, text, text_origin, text_face, text_scale, (127,255,127), text_thickness, cv.LINE_AA)

        cv.imshow("Detected features", image)

    def show_corners_found(self, img, corners, color, window_name="Detected corners", mask=None):
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
            # text_scale = 0.7
            text_scale = 0.5
            text_thickness = 1
            text = f"{i}"
            # text_offset = 20
            text_offset = 10

            text_size, _ = cv.getTextSize(text, text_face, text_scale, text_thickness)
            text_origin = (
                int(center[0] - text_size[0] / 2) + text_offset,
                int(center[1] + text_size[1] / 2) - text_offset
            )

            # cv.circle(image, center, 8, c, cv.FILLED)
            cv.circle(image, center, 4, c, cv.FILLED)
            cv.putText(image, text, text_origin, text_face, text_scale, (127,255,127), text_thickness, cv.LINE_AA)

        cv.imshow(window_name, image)

def main():
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

    corner_detector = FeatureDetector(shi_tomasi_config, hough_circle_config)

    img = cv.imread("test_images/real/frame0001.jpg")

    import glob
    images = [(cv.imread(file), file) for file in sorted(glob.glob("test_images/real/*.jpg"))]

    # Use this to only test the images where the correct corners are not found
    # misdetected_images = [
    #     'test_images/real/frame0121.jpg', 'test_images/real/frame0132.jpg', 'test_images/real/frame0203.jpg',
    #     'test_images/real/frame0228.jpg', 'test_images/real/frame0229.jpg', 'test_images/real/frame0231.jpg',
    #     'test_images/real/frame0258.jpg'
    # ]
    # images = [(cv.imread(file), file) for file in sorted(misdetected_images)]

    features_metric = np.loadtxt("../../data/helipad_dists_origin_center_enu_metric.txt")

    misdetections = []

    import time

    mask_durations = []
    corners_durations = []
    features_durations = []


    for (img, filename) in images:
        # print(filename, end=" ")
        mask_start_time = time.time()
        mask = corner_detector.create_helipad_mask(img, show_masked_img=False)
        mask_duration = time.time() - mask_start_time
        mask_durations.append(mask_duration)

        corners_start_time = time.time()
        corners = corner_detector.find_corners_shi_tomasi(img, mask)
        corners_duration = time.time() - corners_start_time
        corners_durations.append(corners_duration)

        features_start_time = time.time()
        features = corner_detector.find_arrow_and_H(corners, features_metric)
        features_duration = time.time() - features_start_time
        features_durations.append(features_duration)

        # if mask_duration >= 0.25:
        print(f"{filename} Mask time: {mask_duration:.3f} sec Corners time: {corners_duration:.3f} sec Features time: {features_duration:.3f} sec")

        if features is None:
            misdetections.append(filename)
            corner_detector.show_corners_found(img, corners, "red", window_name="Detected features", mask=mask)
            cv.waitKey()
        else:
            corner_detector.show_known_points(img, features, mask=mask)
            cv.waitKey()

    print(f"Mask time: Average: {np.mean(mask_durations):.4f} Median: {np.median(mask_durations):.4f} Max: {np.max(mask_durations):.4f}")
    print(f"Corners time: Average: {np.mean(corners_durations):.4f} Median: {np.median(corners_durations):.4f} Max: {np.max(corners_durations):.4f}")
    print(f"Features time: Average: {np.mean(features_durations):.4f} Median: {np.median(features_durations):.4f} Max: {np.max(features_durations):.2f}")

    print(f"Unable to identify features in {len(misdetections)}/{len(images)} images ({100*(len(images) - len(misdetections))/len(images):.1f}% success)")
    print(misdetections)

    # img = cv.imread("../../data/test_images_real/real_test_2.png")
    # img_gray = corner_detector.preprocess_image(img)
    # cv.imshow("test", img)
    # corners = corner_detector.find_corners_harris(img_gray)
    # corner_detector.show_corners_found(img, corners)
    # mask = corner_detector.create_helipad_mask(img, show_masked_img=False)
    # corners = corner_detector.find_corners_shi_tomasi(img, mask)
    # corner_detector.show_corners_found(img, corners, color="red")

    # features_metric = np.loadtxt("../../data/helipad_dists_origin_center_enu_metric.txt")

    # features = corner_detector.find_arrow_and_H(corners, features_metric)
    # corner_detector.show_known_points(img, features)
    # cv.waitKey()

if __name__ == "__main__":
    main()