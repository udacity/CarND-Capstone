import cv2

def preprocess(input_img):
    # img = cv2.resize(input_img, (800, 600), interpolation=cv2.INTER_CUBIC)
    out = cv2.bilateralFilter(input_img,11,75,75)
    # ----- RGB
    rgb = cv2.cvtColor(out, cv2.COLOR_BGR2RGB)

    # Filter BLUE color
    rgb[ rgb[:,:,2] > 180 ] = 0
    # Filter YELLOW color
    rgb[ (rgb[:,:,1] < 230) & (rgb[:,:,2] > 150) ] = 0
    # Filter RED < color
    rgb[ rgb[:,:,0] < rgb[:,:,1] ] = 0
    rgb[ rgb[:,:,0] < rgb[:,:,2] ] = 0
    # Filter EQUAL channel color
    mask1 = (rgb[:,:,0]-rgb[:,:,1] < 20) | (rgb[:,:,1]-rgb[:,:,0] < 20)
    mask2 = (rgb[:,:,0]-rgb[:,:,2] < 20) | (rgb[:,:,2]-rgb[:,:,0] < 20)
    mask3 = (rgb[:,:,1]-rgb[:,:,2] < 20) | (rgb[:,:,2]-rgb[:,:,1] < 20)
    rgb[ mask1 & mask2 & mask3 ] = 0
    rgb[ mask2 ] = 0

    # ----- Combine channel
    image = rgb[:,:,0]
    _, image = cv2.threshold(image,235,255,cv2.THRESH_BINARY)

    return image

def correctness(idx, prediction):
    # RED
    TRUTH = set()
    TRUTH = TRUTH.union(range(40,51))
    TRUTH = TRUTH.union((50,66,67))
    TRUTH = TRUTH.union(range(116,143))
    # YELLOW
    VAGUE = set()
    VAGUE = VAGUE.union(range(25,40))
    VAGUE = VAGUE.union(range(100,116))
    VAGUE = VAGUE.union(range(177,180))

    if idx in VAGUE:
        return prediction == RED or prediction == UNKNOWN
    elif idx in TRUTH:
        return prediction == RED
    else:
        return prediction != RED
    # return prediction == RED if idx in TRUTH else (prediction != RED or
