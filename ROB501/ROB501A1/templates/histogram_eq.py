import numpy as np

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
    #--- FILL ME IN ---

    # Verify I is grayscale.
    if I.dtype != np.uint8:
        raise ValueError('Incorrect image format!')

    #Initialize arrays for histogram, cdf function and intensity remapping function

    hist = np.zeros(256)
    cdf = np.zeros(256)
    f = np.zeros(256)
    y,x = I.shape

    #Scan image and populate histogram
    for i in range(0,x):
        for j in range(0,y):
            intensity = I[j,i]
            hist[intensity] += 1

    #Calculate and populate cdf function and intensity remapping function
    n = x*y
    cdf[0] = (1/n)*hist[0]
    f[0] = round(cdf[0]*255)
    for i in range(1,256):
        cdf_val = cdf[i-1] + (1/n)*hist[i]
        cdf[i] = cdf_val
        f_val = round(cdf_val*255)
        f[i] = f_val

    #Initialize new image array
    J = np.zeros((y,x))

    #Remap pixel intensities of old image to new image
    for i in range(0,x):
        for j in range(0,y):
            J[j,i] = f[I[j,i]]
    
    #------------------

    return J