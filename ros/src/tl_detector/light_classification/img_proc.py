import cv2
import numpy as np
from os import listdir
from os.path import isfile, join

def normalize_channel(image):
    min_val = np.min(image.ravel())
    max_val = np.max(image.ravel())
    return (image - min_val) / (max_val - min_val)


def detect_light(enhanced_img, thrs):
    ret,thresh_r = cv2.threshold(enhanced_img, thrs, 1.0, cv2.THRESH_BINARY)
    thresh_r = thresh_r.astype('uint8')
    #cv2.imwrite('thresh.png', thresh_r * 255)

    n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(thresh_r)
    for stat in stats:
        width = stat[cv2.CC_STAT_WIDTH]
        height = stat[cv2.CC_STAT_HEIGHT]
        max_dim = max(width, height)
        min_dim = min(width, height)
        if 4 < width < 44 and 4 < height < 44 and float(max_dim) / min_dim < 1.3:
            return True, max_dim
    return False, 0


def analyze_image(image):
    b, g, r = cv2.split(image)
    r = normalize_channel(r.astype(float))
    b = normalize_channel(b.astype(float))
    g = normalize_channel(g.astype(float))
    r_enh = r - b - g
    r_enh = (r_enh + 2.0) / 3.0
    #cv2.imwrite('r_enh.png', r_enh * 255)

    red_seen, red_size = detect_light(r_enh, 0.7)

    g_enh = g - b - r
    g_enh = (g_enh + 2.0) / 3.0
    #cv2.imwrite('g_enh.png', g_enh * 255)

    green_seen, green_size = detect_light(g_enh, 0.7)

    y_enh = r + g - b
    y_enh = (y_enh + 1.0) / 3.0
    #cv2.imwrite('y_enh.png', y_enh * 255)

    yellow_seen, yellow_size = detect_light(y_enh, 0.7)

    if not red_seen and not green_seen and not yellow_seen:
        #print('NO light detected')
        return 0
    if red_size > green_size and red_size > yellow_size:
        #print('red light detected')
        return 1
    if green_size > red_size and green_size > yellow_size:
        #print('green light detected')
        return 2
    if yellow_size > red_size and yellow_size > green_size:
        #print('yellow light detected')
        return 3
    return 0


img_files = [f for f in listdir('./imgs') if isfile(join('./imgs', f))]

for file in img_files:
    print(file)
    img = cv2.imread(join('./imgs/', file))
    analyze_image(img)
    print('----')

