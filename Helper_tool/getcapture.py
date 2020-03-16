import os
import glob
import numpy as np
import matplotlib.pyplot as plt

# Get the all the '*.txt' as I save them in the raw_img file.
img_list = glob.glob(r"raw_img/*.txt")

# Iterate all the raw txt file and convert them into jpeg.

for k,i in enumerate(img_list):
    text_file = open(i, "r")

    # Read the lines, here lines is a list.
    lines = text_file.readlines()
    text_file.close()

    # Line is the first element of lines, as a str[].
    line = lines[0]

    # Pop out the useless string, like '[', ']', and '\n' .
    line = line[1:-2]

    # Convert strings to int, or 'uint8'
    line2 = np.fromstring(line, dtype='uint8', sep=',')

    # Reshape the array to ndarray
    img = line2.reshape((600,800,3))

    # save the image into the img file
    dir = "img/sim_" + str(k) + ".jpeg"
    plt.imsave(dir,img)