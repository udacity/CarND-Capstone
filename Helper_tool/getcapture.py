import numpy as np
import matplotlib.pyplot as plt

# Open whatever file we pull from the simulator,
# 'output.txt' as I save from the simulation.
text_file = open("output.txt", "r")

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

# See if our img display well
plt.imshow(img)
plt.show()
