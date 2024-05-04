import numpy as np
from scipy.ndimage.filters import *

def stereo_disparity_best(Il, Ir, bbox, maxd):
        """
        Best stereo correspondence algorithm.

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
        
        """
        The stereo matching algorithm uses unsharp masking which is an image
        sharpening technique. The algorithm involves first applying a linear
        filter to both the left and right images and then using the sum of
        absolute difference as the matching cost for stereo correspondence.
        The formula for the kernel to give the sharpened image is:
        sharpened = original + (original - blurred) * amount
        where "amount" is a constant multiplier, "original" is a kernel that 
        gives the original image (zeros with 1 at the center), and "blurred" 
        is a kernel that gives a blurred image. In this case it is a uniform
        kernel with 5 pixels. From testing, it seemed that a multiplier of 2
        performed the best. There is information about unsharp masking at
        https://en.wikipedia.org/wiki/Unsharp_masking.
        """
   
        #Size of window, from testing a 15x15 window seemed to work the best, value must be odd
        window_size = 15
        print("Window Size:")
        print(window_size)

        #Size of image
        y,x = Il.shape

        #Constant multiplier for kernel
        multiplier = 2
        print("Multiplier:")
        print(multiplier)

        #Kernel that gives the original image
        original = np.array([[0,0,0], [0,1,0], [0,0,0]])
        #Kernel that gives a blurred image
        blurred = (1/5)*np.array([[0,1,0], [1,1,1], [0,1,0]])
        
        #Kernel used to sharpen image 
        ker = np.add(original,multiplier*np.subtract(original,blurred))

        print("Kernel:")
        print(ker)

        #Pad the left and right images with zeros to obtain an image of same size after filtering
        Il_pad = np.vstack([np.zeros((1,x)),Il,np.zeros((1,x))])
        Il_pad = np.hstack([np.zeros((y+2,1)),Il_pad,np.zeros((y+2,1))])
        Ir_pad = np.vstack([np.zeros((1,x)),Ir,np.zeros((1,x))])
        Ir_pad = np.hstack([np.zeros((y+2,1)),Ir_pad,np.zeros((y+2,1))])

        # Initialize the sharpened images
        Il_sharp = np.zeros((y,x))
        Ir_sharp = np.zeros((y,x))
        
        # Apply the linear filter using cross-correlation
        for i in range(1,x+1):
                for j in range(1,y+1):
                        Il_sharp[j-1,i-1] = np.sum(np.multiply(Il_pad[j-1:j+2,i-1:i+2],ker))
                        Ir_sharp[j-1,i-1] = np.sum(np.multiply(Ir_pad[j-1:j+2,i-1:i+2],ker))

        # Perform stereo matching
        Id = stereo(Il_sharp,Ir_sharp,bbox,maxd,window_size)

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