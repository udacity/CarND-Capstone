import cv2
import numpy as np

class LightDetector():
    
    def orange(self,rgb):
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
    
    def drawCirclesAndGetImages(self,img,camera=False):
        circles,gray,original = self.getCircles(img,camera)
        output = gray.copy()
        h,w = img.shape[:2]
        images=[]
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            index = 0
            for (x, y, r) in circles:
                h-=1
                w-=1
                #if x-r+3>=w or y-r+3>=h:
                    #print("out")
                    #continue
                    
                    #a = 3
                rgb = original[y,x]#original[y-r+3,x-r+3]

                #rgb = original[h-((y+r)%h),w-((x+r)%w)]#simulator
                rgb2 = original[y,x]
                #rgb = img[y+10,x+10] #on all
                #rgb = img[y+7,x-7] #on all
                #rgb2 = img[y-1,x-7]
                #print (rgb)
                crSize = 20
                if self.orange(rgb) or self.orange(rgb2):
                    #cv2.rectangle(output, (x, y), (x, y), (0, 0, 0), -1)
                    crop_img = img[y-crSize:y+crSize, x-crSize:x+crSize]
                    images.append([crop_img,"orange"])
                    #cv2.circle(output, (x, y), r+10, (150, 150, 150), 4)
                    index+=1
                elif self.green(rgb) or self.green(rgb2):
                    #cv2.rectangle(output, (x, y), (x, y), (0, 0, 0), -1)
                    crop_img = img[y-crSize:y+crSize, x-crSize:x+crSize]
                    images.append([crop_img,"green"])
                    #cv2.circle(output, (x, y), r+10, (150, 150, 150), 4)
                    index+=1  
                elif self.red(rgb) or self.red(rgb2):
                    #cv2.rectangle(output, (x, y), (x, y), (0, 0, 0), -1)
                    crop_img = img[y-crSize:y+crSize, x-crSize:x+crSize]
                    images.append([crop_img,"red"])
                    #cv2.circle(output, (x, y), r+10, (150, 150, 150), 4)
                    index+=1
                else:
                    a=0
                    #print(rgb)

                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                #cv2.rectangle(output, (x, y), (x, y), (0, 0, 0), -1)
                #num = 1
                #cv2.rectangle(output, (x, y), (x, y), (0, 128, 255), -1)
        return images,output
    def getLightColor(self,images):
        result = "green"
        for (image,name) in images:
            if name=="red":
                return "red"
            elif name=="orange":
                result="orange"

        return result
