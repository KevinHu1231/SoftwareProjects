# Add other imports from available listing as your see fit.
import os
import sys
import numpy as np
from cv2 import imread, imwrite, resize, imshow, waitKey, destroyAllWindows
from cv2 import GaussianBlur
import cv2
from matplotlib import pyplot as plt

def histogram_eq(I):
    """
    Histogram equalization for greyscale image.
    Perform histogram equalization on the 8-bit greyscale intensity image I
    to produce a contrast-enhanced image J. Full details of the algorithm are
    provided in the Szeliski text.
    Parameters:
    -----------
    I  - Single-band (greyscale) intensity image, 8-bit np.array (i.e., uint8).
    Returns:
    --------
    J  - Contrast-enhanced greyscale intensity image, 8-bit np.array (i.e., uint8).
    """
    # # Verify I is grayscale.
    # if I.dtype != np.uint8:
    #     raise ValueError('Incorrect image format!')

    # qiaoxin2 solution
    debug = False
    J = np.empty(I.shape)
    numPixels = I.shape[0]*I.shape[1]
    bins = np.sort(np.unique(I))

    # Initialize histogram
    hist = {}
    # Create a mapping between intensity and frequency
    for bin in bins:
        hist.update({bin:len(np.argwhere(I==bin))})

    cmf = {}
    # Initialize first bin
    cmf.update({bins[0]:hist.get(bins[0])})
    # Make CMF Mapping
    for bin_index in range(1,len(bins),1):
        last_bin = bins[bin_index - 1]
        current_bin = bins[bin_index]
        # Cumulative Density (Add the current value to the previous sum)
        cmf.update({current_bin:(cmf.get(last_bin)+hist.get(current_bin))})

    # Make J and shift all the intensity values based on cmf
    for row in range(0,I.shape[0],1):
        for column in range(0,I.shape[1],1):
            original_intensity = I[row,column]
            current_cmf = cmf.get(original_intensity)
            ratio = current_cmf/numPixels
            new_intensity = round(255*ratio)
            J[row,column] = new_intensity

    if debug:
        print("Original Image shape:", I.shape)
        print("numPixels: ",numPixels)
        print("histagram: ",hist)
        print("CMF: ",cmf)
    #------------------

    return J

def increase_brightness(img, value=30):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += value

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img

# You can pass additional arguments here by modifying the function signature,
# as long as each additional argument has a default value.
def undo_vignetting(I):
    # Load images
    img_rgb = imread(I)
    # Covert RGB to BGR
    # img_bgr = np.transpose(img,(2,0,1))
    img_bgr = img_rgb[:,:,::-1]
    # np.empty(img_rgb.shape)
    # img_bgr[:,:,0] = img_rgb[:,:,2]
    # img_bgr[:,:,1] = img_rgb[:,:,1]
    # img_bgr[:,:,2] = img_rgb[:,:,0]

    # print("BGR Image",img_bgr.shape)
    height, width = img_bgr.shape[:2]
    # print("Image Shape:",img_bgr.shape)
    # print("The image size is: ", height,width)
    original = img_bgr.copy()
    # imshow("og",original)
    # generating vignette mask using Gaussian kernels
    blur = GaussianBlur(img_bgr,(5,5),0)

    # print("BLUR shape",blur.shape)

    blur = increase_brightness(blur,30)

    # fig1 = plt.figure()
    # plt.imshow(original)
    #
    # fig2 = plt.figure()
    # plt.imshow(blur)
    plt.imshow(blur)
    plt.show()

    return blur

if __name__ == "__main__":
    # Add test code here if you desire.
    print(os.getcwd())
    # img_path = 'rob501_fall_2022_panomania/images/lala.jpg'
    #
    # undo_vignetting(img_path)