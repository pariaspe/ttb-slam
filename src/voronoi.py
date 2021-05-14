#!/usr/bin/env python

import numpy as np
import cv2
import rospy
import matplotlib.pyplot as plt

def generate_voronoi(original_img):
    ''' Loads a binary map and returns the voronoi representation'''
    
    # Load map as an image
    ret, original_img = cv2.threshold(original_img, 0, 1, cv2.THRESH_BINARY_INV)

    # Resize the image for showing purposes

    #dim = (original_img.shape[1] * 10, original_img.shape[0] * 10)
    #original_img = cv2.resize(original_img, dim, interpolation = cv2.INTER_AREA)

    img = original_img.copy()

    size = np.size(img)
    skel = np.zeros(img.shape,img.dtype)


    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))  # Element for morph transformations
    done = False

    # Skelitization
    while not done:
        eroded = cv2.erode(img,element)
        temp = cv2.dilate(eroded,element)
        temp = cv2.subtract(img,temp)
        skel = cv2.bitwise_or(skel,temp)
        img = eroded.copy()
        
        # Stop when the image is fully eroded
        zeros = size - cv2.countNonZero(img)
        if zeros==size:
            done = True

    
    # Image showing

    #cv2.imshow("skel",skel)
    #cv2.imshow("image", original_img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    # Free spaces = 0
    ret, final_img = cv2.threshold(skel, 0, 1, cv2.THRESH_BINARY_INV)
    plt.imshow(final_img, cmap='Greys',  interpolation='nearest')
    plt.savefig('Generated/voronoi.png')
    return final_img


