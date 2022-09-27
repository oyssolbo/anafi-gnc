# File to help with labeling the ground truth coordinates of the arrow and the corners of the H in
# the helipad

import os
import sys
import glob
import itertools
import cv2 as cv
import numpy as np
import pandas as pd

def click_event(event, x, y, flags, param):

    img = param[0]
    filename = param[1]

    # checking for left mouse clicks
    if event == cv.EVENT_LBUTTONDOWN:
        idx = param[2]["current_idx"]
        if idx >= 13:
            print("13 points already added. Not adding more.")
            return

        center = (x,y)
        param[2]["coords"][idx,:] = center
        param[2]["current_idx"] += 1

        color = (0,0,255) # red

        cv.circle(img, center, 4, color, cv.FILLED)

        text_face = cv.FONT_HERSHEY_DUPLEX
        text_scale = 0.3
        text_thickness = 1
        text = f"{center}"
        text_offset = 10

        text_size, _ = cv.getTextSize(text, text_face, text_scale, text_thickness)
        text_origin = (
            int(center[0] - text_size[0] / 2),
            int(center[1] + text_size[1] / 2) - text_offset
        )

        cv.circle(img, center, 4, color, cv.FILLED)
        cv.putText(img, text, text_origin, text_face, text_scale, (127,255,127), text_thickness, cv.LINE_AA)

        cv.imshow("image", img)

def save_labels(df: pd.DataFrame, filename:str):
    idx = list(itertools.chain.from_iterable(zip([f"x{i}" for i in range(13)], [f"y{i}" for i in range(13)])))
    print(f"Saving labels to {output_file}")
    df.to_csv(filename, columns=idx)

# Load images
images = [(cv.imread(file), file) for file in sorted(glob.glob("test_images/real/*.jpg"))]

# Load output file
output_file = "test_images/real/corner_labels.csv"
try:
    labels_df = pd.read_csv(output_file, index_col=0)
except FileNotFoundError:
    print(f"Could not find previous labels file {output_file}, creating a new one.")
    labels_df = pd.DataFrame()

start_image = 0
print(f"Starting from image {start_image}: {images[start_image][1]}")

for i, (img, filename) in enumerate(images[start_image:]):
    header = f"{'='*10} Labeling image: {filename} ({i}/{len(images)}) {'='*10}"
    print(header)

    ans = "r"

    img_with_corners = img.copy()

    labels = {"coords": np.zeros((13,2)), "current_idx": 0}

    while ans != "":
        cv.imshow("image", img_with_corners)
        cv.setMouseCallback("image", click_event, param=(img_with_corners, filename, labels))
        cv.waitKey(0)

        ans = input("Save labels and go to next image [Enter], retry [r] or quit program [q]? ")
        if ans == "r":
            print("Resetting labels on current image")
            img_with_corners = img.copy() # clear labels from image
            labels = {"coords": np.zeros((13,2)), "current_idx": 0} # clear coords
        elif ans == "q":
            cv.destroyAllWindows()
            sys.exit(0)
        else:
            if labels["current_idx"] <= 12:
                print("13 features not present, add remainding")
                ans = "r" # ensure coords are not saved
                continue
            print(f"Labelling complete for image {filename}")

            frame_id = os.path.basename(filename)

            if frame_id in labels_df.index: # replace labels for image
                labels_df.loc[frame_id, :] = labels["coords"].flatten()
            else: # add new label
                idx = list(itertools.chain.from_iterable(zip([f"x{i}" for i in range(13)], [f"y{i}" for i in range(13)])))
                new_label = pd.Series(data=labels["coords"].flatten(), index=idx, name=frame_id)
                labels_df = labels_df.append(new_label, ignore_index=False)

            save_labels(labels_df, output_file)
            print("="*len(header))

cv.destroyAllWindows()