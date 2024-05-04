import numpy as np
from numpy.linalg import inv

def bilinear_interp(I, pt):
    """
    Performs bilinear interpolation for a given image point.

    Given the (x, y) location of a point in an input image, use the surrounding
    four pixels to conmpute the bilinearly-interpolated output pixel intensity.

    Note that images are (usually) integer-valued functions (in 2D), therefore
    the intensity value you return must be an integer (use round()).

    This function is for a *single* image band only - for RGB images, you will 
    need to call the function once for each colour channel.

    Parameters:
    -----------
    I   - Single-band (greyscale) intensity image, 8-bit np.array (i.e., uint8).
    pt  - 2x1 np.array of point in input image (x, y), with subpixel precision.

    Returns:
    --------
    b  - Interpolated brightness or intensity value (whole number >= 0).
    """
    #--- FILL ME IN ---

    if pt.shape != (2, 1):
        raise ValueError('Point size is incorrect.')

    #Get dimensions of image

    width,height = I.shape

    x = pt[0,0]
    y = pt[1,0]

    #Get the four nearest pixel coordinates

    x1 = int(x)
    y1 = int(y)

    x1 = min(x1, height-2)
    y1 = min(y1, width-2)

    x2 = x1 + 1
    y2 = y1 + 1

    #Calculate intensities of the four nearest pixels

    I11 = I[y1,x1]
    I12 = I[y1,x2]
    I21 = I[y2,x1]
    I22 = I[y2,x2]

    x_x1 = x - x1
    y_y1 = y - y1

    #Calculate the intensity using bilinear interpolation

    b = I11*(1-x_x1)*(1-y_y1) + I12*x_x1*(1-y_y1) + I21*(1-x_x1)*y_y1 + I22*x_x1*y_y1

    #Make sure that intensity values are valid

    if b<0:
        b = 0
    elif b>255:
        b = 255
    else:
        b = round(b)

    #------------------

    return b

    