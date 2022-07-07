import numpy as np
import cv2 as cv
import glob

pose_l=[]

mtx=np.array([[1010.80109, 0.00000000, 1021.91560],
                [0.00000000, 995.496826, 770.488484],
                [0.00000000, 0.00000000, 1.00000000]])
dist=np.array([[ 0.10101934, -0.11059437, -0.00038653, -0.00049741,  0.05725655]])

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((8*5,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:5].T.reshape(-1,2)


for image in glob.glob('*.jpg'):
    img = cv.imread(image)
    gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(gray, (8,5),None)

    if ret == True:

        corners2 = cv.cornerSubPix(gray,corners,(11,11),(-1,-1), criteria)

        # Find the rotation and translation vectors.
        x, rvecs, tvecs,y= cv.solvePnPRansac(objp, corners2, mtx, dist)
        rvecs_r=cv.Rodrigues(rvecs)[0]
        row=np.array([0,0,0,1])

        T=np.c_[rvecs_r,tvecs*0.04]
        T=np.vstack([T,row])
        pose_l.append(T)

    print("Rotation_rod: \n",rvecs_r,"\n",
          "Translation: \n",tvecs,"\n"
          "Translation: \n", T)


cv.destroyAllWindows()
