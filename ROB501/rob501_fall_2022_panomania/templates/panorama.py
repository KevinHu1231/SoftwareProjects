# Add other imports from available listing as your see fit.
import sys
import numpy as np
from cv2 import imread, imwrite, resize, imshow, waitKey, destroyAllWindows
from cv2 import SIFT_create, BRISK_create, ORB_create
from cv2 import BFMatcher_create, FlannBasedMatcher_create
from cv2 import drawKeypoints, drawMatches
from cv2 import findHomography, GaussianBlur, RANSAC
from mosaic_images import mosaic_images
from undo_vignetting import undo_vignetting
def create_panorama(image_files):
    # Load images.
    I1 = imread(image_files[0])
    I2 = imread(image_files[1])
    I3 = imread(image_files[2])
    I4 = imread(image_files[3])
    I5 = imread(image_files[4])
    I6 = imread(image_files[5])
    I7 = imread(image_files[6])

    print("Images Loaded")

    images = [I1,I2,I3,I4,I5,I6,I7]

    # Mosaic all images.
    
    # Use image 1 as base image
    # Mosaic image 1 with image 2 and set the combined image as the new base image
    # Continue until all images are combined together.

    I_base = I1
    for i in range(1,7):
        I_result = mosaic_images(I_base,images[i])
        I_base = I_result
    
    Ipano = I_result

    # Undo vignetting if desired.
    Ipano = undo_vignetting(Ipano)

    # Write out the result as a PNG.
    # Resize image to desired image size
    Ipano_resized = resize(Ipano,(1250,3000))
    Ipano = Ipano_resized
    imwrite(Ipano)
    imshow('Ipano',Ipano)

    waitKey()
    destroyAllWindows()

if __name__ == "__main__":
    # Make sure the right number of input args are provided.
    if len(sys.argv) != 8:
        sys.exit(-1, "Too few command line argmuments.")

    create_panorama(sys.argv[1:])
    #Command sub_image_01_dark.png sub_image_02_dark.png sub_image_03_dark.png sub_image_04_dark.png sub_image_05_dark.png sub_image_06_dark.png sub_image_07_dark.png