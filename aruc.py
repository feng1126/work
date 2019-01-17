import numpy as np
import cv2
import cv2.aruco as aruco


'''
    drawMarker(...)
        drawMarker(dictionary, id, sidePixels[, img[, borderBits]]) -> img
'''

aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
print(aruco_dict)
# second parameter is id number
# last parameter is total image size
img = aruco.drawMarker(aruco_dict, 26, 600)
cv2.imwrite("26.png", img)

cv2.imshow('frame',img)
cv2.waitKey(0)




while(True):
    # Capture frame-by-frame

    #print(frame.shape) #480x640
    # Our operations on the frame come here
    gray = img
    #gray = cv2.imread("/home/feng/aruco42.png")
    #aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters =  aruco.DetectorParameters_create()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(ids)

    #It's working.
    # my problem was that the cellphone put black all around it. The alrogithm
    # depends very much upon finding rectangular black blobs

    gray = aruco.drawDetectedMarkers(gray, corners)

    #print(rejectedImgPoints)
    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture

cv2.destroyAllWindows()
