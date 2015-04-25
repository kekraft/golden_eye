#!/usr/bin/env python 

import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import copy

def main(x1, x2, y1, y2):
    img = cv2.imread('saved.jpg',0)
    print img.shape
    # refined_blob_detection(img)
    # cannyEdgeDetection(img)

    # Select region of interest
    roi = img[y1:y2, x1:x2]
    cv2.imwrite("roi.jpg", roi)

    plt.subplot(121),plt.imshow(img),plt.title('Original')
    plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(roi),plt.title('Region of interest')
    plt.xticks([]), plt.yticks([])
    plt.show()  
    feat_detection(roi)

    # blurring 
    # blur = cv2.blur(roi,(5,5))
    blur = cv2.GaussianBlur(roi,(5,5),0)  
    # plt.subplot(121),plt.imshow(roi),plt.title('Original Cut')
    # plt.xticks([]), plt.yticks([])
    # plt.subplot(122),plt.imshow(blur),plt.title('Blurred')
    # plt.xticks([]), plt.yticks([])
    # plt.show()
    feat_detection(blur)

    sharp = cv2.addWeighted(roi,0.7,blur,0.3,0)
    # plt.subplot(121),plt.imshow(roi),plt.title('Original Cut')
    # plt.xticks([]), plt.yticks([])
    # plt.subplot(122),plt.imshow(sharp),plt.title('Sharpened')
    # plt.xticks([]), plt.yticks([])
    # plt.show()

    # try to sharpen again
    # blur = cv2.blur(roi,(5,5))
    blur = cv2.GaussianBlur(sharp,(5,5),0)  
    # plt.subplot(121),plt.imshow(sharp),plt.title('Sharpened')
    # plt.xticks([]), plt.yticks([])
    # plt.subplot(122),plt.imshow(blur),plt.title('Sharp Blurred')
    # plt.xticks([]), plt.yticks([])
    # plt.show()
    feat_detection(blur)

    sharp2 = cv2.addWeighted(sharp,0.7,blur,0.3,0)
    feat_detection(sharp2)
    # plt.subplot(121),plt.imshow(sharp),plt.title('Sharpened')
    # plt.xticks([]), plt.yticks([])
    # plt.subplot(122),plt.imshow(sharp2),plt.title('Extra Sharpened')
    # plt.xticks([]), plt.yticks([])
    # plt.show()

    # bilateral filtering
    blur = cv2.bilateralFilter(sharp2,9,75,75)
    # plt.imshow(blur)
    # plt.show()
    feat_detection(blur)


    # contours_example(roi)

# def contours_example(img):
#     h, w = img.shape[:2]

#     contours0, hierarchy = cv2.findContours( img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#     contours = [cv2.approxPolyDP(cnt, 3, True) for cnt in contours0]


#     update(3, h, w, contours, hierarchy)
#     cv2.createTrackbar( "levels+3", "contours", 3, 7, update )
#     cv2.imshow('image', img)
#     0xFF & cv2.waitKey()
#     cv2.destroyAllWindows()

# def update(levels, h, w, contours, hierarchy):
#     vis = np.zeros((h, w, 3), np.uint8)
#     levels = levels - 3
#     cv2.drawContours( vis, contours, (-1, 3)[levels <= 0], (128,255,255),
#         3, cv2.CV_AA, hierarchy, abs(levels) )
#     cv2.imshow('contours', vis)

def feat_detection(img):
    print "Feature detection"
    surf_detect(img)
    detect_circles(img)
    refined_blob_detection(img)
    cannyEdgeDetection(img)
    largest_contours(img)

def sharpen(img, gauss_kernel=5, orig_weight=.7):
    blur = cv2.GaussianBlur(img,(gauss_kernel,gauss_kernel),0)  
    sharp = cv2.addWeighted(img,orig_weight,blur,1-orig_weight,0)
    return sharp

def surf_detect(img):
    # surf detection
    surf = cv2.SURF(500)
    kp, des = surf.detectAndCompute(img,None)
    img_with_kp = cv2.drawKeypoints(img,kp,None,(255,0,0),4)
    plt.imshow(img_with_kp)
    plt.title("surf detection")
    plt.show()

def detect_circles(img):    
    # img = cv2.medianBlur(img, 5)
    # grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    circles = cv2.HoughCircles(img, cv2.cv.CV_HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)
    # circles = cv2.HoughCircles(img, cv2.cv.CV_HOUGH_GRADIENT, 1.2, 75)
   
#     circles = np.uint(np.around(circles, decimals=2))
    if circles is not None:
            
#         circles = np.uint(circles, decimals=2)
        circles = np.round(circles[0, :]).astype("int")
        
        
        for (x, y, r) in circles:
            # outer circle
            cv2.circle(img, (x, y), r, (0,255,0), 4)
            # center of circle
#             cv2.circle(grayImg, ], i[1]), 2, (0,0,255), 3)
#          
        # cv2.imshow('detected circles', np.hstack([img]))
        plt.imshow(np.hstack([img]))
        plt.title("Circles")
        plt.show()
    
    else:
        # cv2.imshow('detected circles', img) 
        plt.imshow(img)
        plt.title("Circles")
        plt.show()

        # cv2.waitKey()

def blob_detection(img):

    # Set up the detector with default parameters.
    detector = cv2.SimpleBlobDetector()
     
    # Detect blobs.
    keypoints = detector.detect(img)
     
    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
     
    # Show keypoints
    # cv2.imshow("Keypoints", im_with_keypoints)
    # cv2.waitKey()
    plt.imshow(im_with_keypoints)
    plt.title("Blobs")
    plt.show()

def largest_contours(image):
    # convert the image to grayscale, blur it, and find edges
    # in the image
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = image
    gray = cv2.bilateralFilter(gray, 11, 17, 17)
    gray = cv2.blur(gray,(3,3))
    edged = cv2.Canny(gray, 30, 200)

    # cv2.imshow("Blurred Edged Scene", edged)
    # cv2.waitKey(1)
    plt.imshow(edged)
    plt.title("Blurred edged scene")
    plt.show()

    # find contours in the edged image, keep only the largest
    # ones, and initialize our screen contour
    (cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]
    screenCnt = None

    # loop over our contours
    # for c in cnts:
    #     # approximate the contour
    #     peri = cv2.arcLength(c, True)
    #     approx = cv2.approxPolyDP(c, 0.02 * peri, True)
 
    #     # if our approximated contour has four points, then
    #     # we can assume that we have found our screen
    #     if len(approx) == 4:
    #         screenCnt = approx
    #         break
    
    if len(cnts) > 0:
        main_cnt = cnts[0]
        perimeter = cv2.arcLength(main_cnt,True)

        epsilon = 0.2*cv2.arcLength(main_cnt,True)
        approx = cv2.approxPolyDP(main_cnt,epsilon,True)

        image_copy = copy.deepcopy(image)
        cv2.drawContours(image_copy, [main_cnt], -1, (0, 255, 0), 3)
        print main_cnt
        # cv2.imshow("Pong Table approx", image_copy)
        # cv2.waitKey()


        # cv2.drawContours(image, [main_cnt], -1, (0, 255, 0), 3)
        # cv2.imshow("Pong Table Main contour", image)
        # cv2.waitKey()

        plt.imshow(image_copy)
        plt.title("Largest contour")
        plt.show()


def cannyEdgeDetection(img):
    edges = cv2.Canny(img, 100, 200)
    
    # display orignal and new
    plt.subplot(121),plt.imshow(img,cmap = 'gray')
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(edges,cmap = 'gray')
    plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
    plt.show()    
        

    return edges

def refined_blob_detection(img):
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()
     
    # Change thresholds
    params.minThreshold = 1;
    params.maxThreshold = 2000;
     
    # Filter by Area.
    params.filterByArea = True
    # params.minArea = 1500
    params.minArea = 1
     
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 1
     
    # Filter by Convexity
    # params.filterByConvexity = True
    # params.minConvexity = 0.5
     
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.01
     
    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        detector = cv2.SimpleBlobDetector(params)
    else : 
        detector = cv2.SimpleBlobDetector_create(params)

     # Detect blobs.
    keypoints = detector.detect(img)
     
    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
     
    # # Show keypoints
    # cv2.imshow("Refined Blob Detection", im_with_keypoints)
    # cv2.waitKey(1)
    plt.imshow(im_with_keypoints)
    plt.title("Refined Blob Detection")
    plt.show()

if __name__ == '__main__':
    
    # try:
    x1 = int(sys.argv[1])
    x2 = int(sys.argv[2])
    y1 = int(sys.argv[3])
    y2 = int(sys.argv[4])

    main(x1, x2, y1, y2)

    # except Exception, e:
    #   print '[Error]: Crop Dimensions Not Input\n\'sift.py\' [x1] [x2] [y1] [y2]'
    

    