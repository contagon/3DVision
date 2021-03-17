import numpy as np
import cv2
import os
import argparse
np.set_printoptions(suppress=True) 

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 300, 1e-7)

def find_params(files, idxs, output=True):
    img_corners = []
    real_corners = []
    for file in files:
        # Read in image
        image = cv2.imread(file)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Find and improve corners
        ret, corners = cv2.findChessboardCorners(image, board_size, None)
        if ret:
            corners = cv2.cornerSubPix(image, corners, (11,11), (-1,-1), criteria)
            # sometimes they're upside down, fix that
            if corners[0,0,0] > corners[-1,0,0]:
                corners = corners[::-1]

            img_corners.append(corners)
            real_corners.append(idxs)
        else:
            print("Couldn't find corners in ", file)

        if args['display']:
            temp = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(temp, board_size, corners, ret)
            cv2.imshow('corners', temp)
            cv2.waitKey()

    # Do actual calibration
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(real_corners, img_corners, image.shape[::-1], None, None)

    if output:
        print("Intrinsic Parameters:")
        print(mtx)
        print("\nDistortion Params: ")
        print(dist.T)

        mean_error = 0
        for i in range(len(real_corners)):
            img_corners2, _ = cv2.projectPoints(real_corners[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(img_corners[i], img_corners2, cv2.NORM_L2)/len(img_corners2)
            mean_error += error
        mean_error /= len(real_corners)
        print('\nMean Error:')
        print(mean_error)


    return mtx, dist, img_corners

if __name__ == "__main__":
    # Parse through arguments
    parser = argparse.ArgumentParser(description="Camera Calibrater")
    parser.add_argument("-o", "--outfile", type=str, default="results/params.npz", help="Location to save params")
    parser.add_argument("-s", "--stereo_folder", type=str, default="calibration/Stereo", help="Folder to load images from")
    parser.add_argument("-l", "--left_folder", type=str, default="calibration/Left_Cali", help="Folder to load images from")
    parser.add_argument("-r", "--right_folder", type=str, default="calibration/Right_Cali", help="Folder to load images from")
    parser.add_argument("-w", "--width", type=int, default=10, help="# of corners wide")
    parser.add_argument("-t", "--height", type=int, default=7, help="# of corners height (tall)")
    parser.add_argument("--display", action="store_true", help="Display found corners")
    parser.add_argument("-d", "--distance", type=float, default=3.88, help="Distance between corners")
    args = vars(parser.parse_args())

    # set up board
    board_size = (args['width'], args['height'])
    idxs = np.zeros((board_size[0]*board_size[1],3), np.float32)
    idxs[:,:2] = np.mgrid[0:board_size[0],0:board_size[1]].T.reshape(-1,2)
    idxs *= args['distance']

    # find left params
    print("\nStarting left camera...")
    files = [os.path.join(args['left_folder'], f) for f in os.listdir(args['left_folder'])]
    mtx_l, dist_l, _ = find_params(files, idxs)

    # find right params
    print("\nStarting right camera...")
    files = [os.path.join(args['right_folder'], f) for f in os.listdir(args['right_folder'])]
    mtx_r, dist_r, _ = find_params(files, idxs)

    # get all imgpts of stereo imgs
    print("\nStarting stereo calibration...")
    files = [os.path.join(args['stereo_folder'], f) for f in os.listdir(args['stereo_folder'])]
    files_l = sorted([f for f in files if 'L' in f])
    files_r = sorted([f for f in files if 'R' in f])
    _, _, imgpts_l = find_params(files_l, idxs, output=False)
    _, _, imgpts_r = find_params(files_r, idxs, output=False)

    # stereo calibration
    real_pts = [idxs]*len(imgpts_l)
    shape = cv2.imread(files[0]).shape[::-1][1:]
    ret, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(real_pts, imgpts_l, imgpts_r, 
                                                        mtx_l, dist_l, mtx_r, dist_r, shape,
                                                        criteria=criteria, flags=cv2.CALIB_FIX_INTRINSIC)
    print("Rotation:")
    print(R)
    print("\nTranslation: ")
    print(T)
    print("\nEssential: ")
    print(E)
    print("\nFundamental: ")
    print(F)    

    # get rectification params
    R_l, R_r, P_l, P_r, Q, _, _ = cv2.stereoRectify(mtx_l, dist_l, mtx_r, dist_r, shape, R, T)

    # save params
    np.savez(args['outfile'], mtx_l=mtx_l, dist_l=dist_l, mtx_r=mtx_r, dist_r=dist_r, R=R, T=T, E=E, F=F, Q=Q, R_l=R_l, P_l=P_l, R_r=R_r, P_r=P_r)