import cv2
import numpy as np
import os
    
def partA():
    file = open('../Generated/stats.csv','w') 
    i=0
    for x in os.listdir('../Images'):
        image = cv2.imread('../Images/'+str(x),1)
        cv2.imshow(x,image)
        rows, cols, ch = image.shape
        r =int(rows/2)
        c =int(cols/2)
        B, G, R = image[r,c,:]
        file.write(x+','+str(rows)+','+str(cols)+','+str(ch)+','+str(B)+','+str(G)+','+str(R)+'\n')
        i=i+1
    file.close()

def partB():
    cat = cv2.imread('../Images/cat.jpg',1)
    cat[:,:,0] = 0
    cat[:,:,1] = 0
    cv2.imwrite('../Generated/cat_red.jpg',cat)

def partC():
    flowers = cv2.imread('../Images/flowers.jpg',1)
    B, G, R = cv2.split(flowers)
    A = 127*np.ones(R.shape, R.dtype)
    flowers = cv2.merge((B,G,R,A))
    cv2.imwrite('../Generated/flowers_alpha.png',flowers)

def partD():
    horse = cv2.imread('../Images/horse.jpg',1)
    B, G, R = cv2.split(horse)
    R=0.3*np.array(R)
    G=0.59*np.array(G)
    B=0.11*np.array(B)
    I = np.add( R, np.add( G, B ) )
    horse = cv2.merge((I, I, I))
    cv2.imwrite('../Generated/horse_gray.jpg',horse)

partA()
partB()
partC()
partD()
