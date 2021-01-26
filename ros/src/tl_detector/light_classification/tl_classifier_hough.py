import cv2
import numpy as np

class LightDetector():
    
    def yellow(self,rgb):
        return rgb[0]>225 and rgb[1]>100 and rgb[2]<160
    
    def green(self,rgb):
        r=int(rgb[0])
        g=int(rgb[1])
        b=int(rgb[2])
        return (g-r > 40) and(g-b > 40)
    
    def red(self,rgb):
        r=int(rgb[0])
        g=int(rgb[1])
        b=int(rgb[2])
        return (r-g > 40) and (r-b > 40)
    
    def getColor(self,img):
        output = img.copy()
        
    def getCircles(self,img,camera):
        original=img
        kernel = None
        #kernel = np.ones((4,4),np.float32)/6
        
        if camera:
            #img = self.decrease_brightness(img,60)
            img = cv2.medianBlur(img,7)
            #kernel = np.ones((4,4),np.float32)/16
        else:
            img = cv2.medianBlur(img,7)
            #kernel = np.ones((5,5),np.float32)/22
        #img = cv2.bitwise_not(img)
        #img = cv2.filter2D(img,-1,kernel)
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if camera:
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT,1,5,param1=170,param2=10,minRadius=4,maxRadius=14)
        else:
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT,1,5,param1=170,param2=18,minRadius=0,maxRadius=30)
        return circles,img,original
    def printImages(self,images):
            if len (images)<1:
                print("None")
            else:
                s = len(images)
                s+=1
                f, ax1 = plt.subplots(s, 1, figsize=(20, 20))
                f.tight_layout()
                ind=0
                for i in images:
                    ax1[ind].imshow(i[0])
                    ax1[ind].set_title(i[1])
                    ind+=1
                    
    def decrease_brightness(self,img, value=30):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        lim = 255 - value
        v[v > lim] = 255
        v[v <= lim] -= value

        final_hsv = cv2.merge((h, s, v))
        img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
        return img
    
    def drawCirclesAndGetImages(self,img,camera=False,withImage=False):
        circles,gray,original = self.getCircles(img,camera)
        output=None
        if withImage:
            output = gray.copy()
        h,w = img.shape[:2]
        images=[]
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                if(y>=h or x>=w):
                    continue
                rgb = original[y,x]
                #rgb2 = original[y,x] optional 
                
                crSize = 20
                if self.yellow(rgb):# or self.orange(rgb2):
                    #cv2.rectangle(output, (x, y), (x, y), (0, 0, 0), -1)
                    if(withImage):
                        crop_img = img[y-crSize:y+crSize, x-crSize:x+crSize]
                        images.append([crop_img,"yellow"])
                    else:
                        images.append([None,"yellow"])
                    #cv2.circle(output, (x, y), r+10, (150, 150, 150), 4)
                elif self.green(rgb):# or self.green(rgb2):
                    #cv2.rectangle(output, (x, y), (x, y), (0, 0, 0), -1)
                    if(withImage):
                        crop_img = img[y-crSize:y+crSize, x-crSize:x+crSize]
                        images.append([crop_img,"green"])
                    else:
                        images.append([None,"green"])
                    #cv2.circle(output, (x, y), r+10, (150, 150, 150), 4) 
                elif self.red(rgb):# or self.red(rgb2):
                    #cv2.rectangle(output, (x, y), (x, y), (0, 0, 0), -1)
                    if(withImage):
                        crop_img = img[y-crSize:y+crSize, x-crSize:x+crSize]
                        images.append([crop_img,"red"])
                    else:
                        images.append([None,"red"])
                    break#if red found one enough
                    
                # draw the circle in the output image, then draw a grey rectangle
                #cv2.circle(output, (x, y), r+10, (150, 150, 150), 4)
                #corresponding to the center of the circle
                #cv2.rectangle(output, (x, y), (x, y), (0, 0, 0), -1)
        return images,output
    def getLightColor(self,images):
        result = "green"
        for (image,name) in images:
            if name=="red":
                return "red"
            elif name=="yellow":
                result="yellow"

        return result
