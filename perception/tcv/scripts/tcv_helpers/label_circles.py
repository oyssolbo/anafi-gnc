import os
import sys
import glob
import cv2 as cv
import numpy as np
import pandas as pd

drawing = False # true if mouse is pressed
ix,iy = -1,-1

# Create a function based on a CV2 Event (Left button click)
def draw_circle(event,x,y,flags,param):
    global ix,iy,drawing

    img = param[0]
    filename = param[1]

    if event == cv.EVENT_LBUTTONDOWN:
        drawing = True
        # we take note of where that mouse was located
        ix,iy = x,y
        cv.imshow("image", img)

    elif event == cv.EVENT_MOUSEMOVE:
        drawing == True

    elif event == cv.EVENT_LBUTTONUP:
        radius = int(np.math.sqrt( ((ix-x)**2)+((iy-y)**2)))

        # Store result
        param[2][0] = ix
        param[2][1] = iy
        param[2][2] = radius

        img_with_circle = img.copy()

        center = (ix,iy)

        cv.circle(img_with_circle,center,radius, (127,255,127), thickness=3)

        text_face = cv.FONT_HERSHEY_DUPLEX
        text_scale = 1
        text_thickness = 2
        text = f"o: {center} r: {radius}"
        text_offset = 10

        text_size, _ = cv.getTextSize(text, text_face, text_scale, text_thickness)
        text_origin = (
            int(center[0] - text_size[0] / 2),
            int(center[1] + text_size[1] / 2) - text_offset
        )

        cv.putText(img_with_circle, text, text_origin, text_face, text_scale, (0,0,255), text_thickness, cv.LINE_AA)

        cv.imshow("image", img_with_circle)
        drawing = False

def save_labels(df: pd.DataFrame, filename:str):
    print(f"Saving labels to {output_file}")
    df.to_csv(filename, columns=["x", "y", "r"])

# Load images
images = [(cv.imread(file), file) for file in sorted(glob.glob("test_images/real/*.jpg"))]

start_image = 0
print(f"Starting from image {start_image}: {images[start_image][1]}")

# Load output file
output_file = "test_images/real/circle_labels.csv"
try:
    labels_df = pd.read_csv(output_file, index_col=0)
except FileNotFoundError:
    print(f"Could not find previous labels file {output_file}, creating a new one.")
    labels_df = pd.DataFrame()

for (img, filename) in images[start_image:]:
    header = f"{'='*10} Labeling image: {filename} {'='*10}"
    print(header)

    ans = "r"

    label = np.zeros(3) # format: [x,y,r]

    while ans != "":
        cv.imshow("image", img)
        cv.setMouseCallback("image", draw_circle, param=(img, filename, label))
        cv.waitKey(0)

        ans = input("Save labels and go to next image [Enter], retry [r] or quit program [q]? ")
        if ans == "r":
            print("Resetting labels on current image")
            label = np.zeros(3) # clear coords
        elif ans == "q":
            cv.destroyAllWindows()
            sys.exit(0)
        else:
            if np.count_nonzero(label) == 0:
                print("No circle drawn, add one.")
                ans = "r"
                continue
            print(f"Labelling complete for image {filename}")
            frame_id = os.path.basename(filename)
            if frame_id in labels_df.index: # replace label for image
                labels_df.loc[frame_id, "x"] = label[0]
                labels_df.loc[frame_id, "y"] = label[1]
                labels_df.loc[frame_id, "r"] = label[2]
            else: # add new label
                new_label = pd.Series(data={"x":label[0], "y":label[1], "r":label[2]}, name=frame_id)
                labels_df = labels_df.append(new_label, ignore_index=False)

            save_labels(labels_df, output_file)
            print("="*len(header))

cv.destroyAllWindows()
