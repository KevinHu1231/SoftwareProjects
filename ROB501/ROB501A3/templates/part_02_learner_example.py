import numpy as np
import matplotlib.pyplot as plt
from imageio import imread
from stereo_disparity_score import stereo_disparity_score
from stereo_disparity_best import stereo_disparity_best, SAD
for i in range(0,2):
    if i == 0:
        # Load the stereo images and ground truth.
        Il = imread("../images/cones/cones_image_02.png", as_gray = True)
        Ir = imread("../images/cones/cones_image_06.png", as_gray = True)

        # The cones and teddy datasets have ground truth that is 'stretched'
        # to fill the range of available intensities - here we divide to undo.
        It = imread("../images/cones/cones_disp_02.png",  as_gray = True)/4.0

        # Load the appropriate bounding box.
        bbox = np.load("../data/cones_02_bounds.npy")

        Id = stereo_disparity_best(Il, Ir, bbox, 55)
        N, rms, pbad = stereo_disparity_score(It, Id, bbox)
        print("Valid pixels: %d, RMS Error: %.2f, Percentage Bad: %.2f" % (N, rms, pbad))
        plt.imshow(Id, cmap = "gray")
        plt.show()
    elif i == 1:
        # Load the stereo images and ground truth.
        Il = imread("../images/teddy/teddy_image_02.png", as_gray = True)
        Ir = imread("../images/teddy/teddy_image_06.png", as_gray = True)

        # The cones and teddy datasets have ground truth that is 'stretched'
        # to fill the range of available intensities - here we divide to undo.
        It = imread("../images/teddy/teddy_disp_02.png",  as_gray = True)/4.0

        # Load the appropriate bounding box.
        bbox = np.load("../data/teddy_02_bounds.npy")

        Id = stereo_disparity_best(Il, Ir, bbox, 55)
        N, rms, pbad = stereo_disparity_score(It, Id, bbox)
        print("Valid pixels: %d, RMS Error: %.2f, Percentage Bad: %.2f" % (N, rms, pbad))
        plt.imshow(Id, cmap = "gray")
        plt.show()
