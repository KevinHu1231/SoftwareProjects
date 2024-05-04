import numpy as np
from scipy.ndimage.filters import *

def stereo_disparity_fast(Il, Ir, bbox, maxd):
        """
        Fast stereo correspondence algorithm.

        This function computes a stereo disparity image from left stereo 
        image Il and right stereo image Ir. Only disparity values within
        the bounding box region (inclusive) are evaluated.

        Parameters:
        -----------
        Il    - Left stereo image, m x n pixel np.array, greyscale.
        Ir    - Right stereo image, m x n pixel np.array, greyscale.
        bbox  - 2x2 np.array, bounding box, relative to left image, from top left
        corner to bottom right corner (inclusive). (x, y) in columns.
        maxd  - Integer, maximum disparity value; disparities must be within zero
        to maxd inclusive (i.e., don't search beyond maxd).

        Returns:
        --------
        Id  - Disparity image (map) as np.array, same size as Il.
        """
        # Hints:
        #
        #  - Loop over each image row, computing the local similarity measure, then
        #    aggregate. At the border, you may replicate edge pixels, or just avoid
        #    using values outside of the image.
        #
        #  - You may hard-code any parameters you require in this function.
        #
        #  - Use whatever window size you think might be suitable.
        #
        #  - Optimize for runtime AND for clarity.

        #--- FILL ME IN ---

        # Code goes here...

        #Size of window, from testing a 15x15 window seemed to work the best, value must be odd
        window_size = 15

        print("Window Size:")
        print(window_size)

        #Perform stereo matching
        Id = stereo(Il,Ir,bbox,maxd,window_size)

        #------------------

        correct = isinstance(Id, np.ndarray) and Id.shape == Il.shape

        if not correct:
                raise TypeError("Wrong type or size returned!")

        return Id

# Helper function that calculated the SAD of two images patches of the same size
def SAD(patch1, patch2):
        if patch1.shape != patch2.shape:
                return 0
        else:
                sad = np.sum(np.absolute(np.subtract(patch1,patch2)))
        return sad

# Function that performs stereo matching
def stereo(Il,Ir,bbox,maxd,window_size):

        # Shape of image
        y,x = Il.shape

        # Top left corner and bottom right corner x and y locations
        tlx, tly, brx, bry = bbox[0,0], bbox[1,0], bbox[0,1], bbox[1,1]

        # Initialize disparity image
        Id = np.zeros(Il.shape)

        # Check if window size is odd
        if (window_size%2 == 1):
                offset = int((window_size-1)/2)

                # Ensure that window does not go outside the left image at any point
                if (offset<=tlx) and (offset<=tly) and (offset<=(y-1-bry)) and (offset<=(x-1-brx)):

                        # Loop through every pixel in left image
                        for i in range(tlx,brx+1):
                                for j in range(tly,bry+1):

                                        # Record best match x coordinate and its SAD
                                        best_match = None
                                        best_sad = np.inf

                                        # Portion of left image within window with the pixel location at the center
                                        feature = Il[j-offset:j+offset+1,i-offset:i+offset+1]

                                        # Loop through every pixel in the right image with the same x coordinate
                                        for k in range(max(offset,i-maxd),i):

                                                # Portion of right image within window with the pixel location at the center
                                                matched = Ir[j-offset:j+offset+1,k-offset:k+offset+1]

                                                # Calculate SAD
                                                sad = SAD(feature,matched)

                                                # Update SAD if it is better the currently recorded value
                                                if sad<best_sad:
                                                        best_sad = sad
                                                        best_match = k
                                                        
                                        # Set the corresponding pixel on the disparity map to the calculated disparity
                                        Id[j,i] = i-best_match
        return Id